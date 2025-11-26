package control

import (
	"context"
	"machine/usb/hid/joystick"
	"sync"
	"time"

	"tinygo.org/x/drivers/mcp2515"

	"github.com/SWITCHSCIENCE/ffb_steering_controller/motor"
	"github.com/SWITCHSCIENCE/ffb_steering_controller/pid"
	"github.com/SWITCHSCIENCE/ffb_steering_controller/settings"
	"github.com/SWITCHSCIENCE/ffb_steering_controller/utils"
)

var (
	ph       = pid.NewPIDHandler()
	limitS16 = utils.Limit(-32767, 32767)
	js       = joystick.UseSettings(joystick.Definitions{
		ReportID:     1,
		ButtonCnt:    0,
		HatSwitchCnt: 0,
		AxisDefs: []joystick.Constraint{
			{MinIn: -32767, MaxIn: 32767, MinOut: -32767, MaxOut: 32767}, // X-Axis
			{MinIn: -32767, MaxIn: 32767, MinOut: -32767, MaxOut: 32767}, // Y-Axis
			{MinIn: -32767, MaxIn: 32767, MinOut: -32767, MaxOut: 32767},
			{MinIn: -32767, MaxIn: 32767, MinOut: -32767, MaxOut: 32767}, // Rx-Axis
			{MinIn: -32767, MaxIn: 32767, MinOut: -32767, MaxOut: 32767}, // Ry-Axis
		},
	}, ph.RxHandler, ph.SetupHandler, pid.Descriptor)
)

type Joystick interface {
	SetHat(index int, dir joystick.HatDirection)
	SetButton(index int, push bool)
	SetAxis(index int, v int)
	SendState()
}

type Wheel struct {
	Joystick
	calc                   func() []int32
	can                    *mcp2515.Device
	mu                     sync.RWMutex
	coggingTorqueCancel    int32
	viscosity              int32
	softLockForceMagnitude int32
	fit                    func(x int32) int32
	limitForce             func(x int32) int32
	rx, ry                 int32
}

func NewWheel(can *mcp2515.Device) *Wheel {
	w := &Wheel{
		Joystick: js,
		calc:     ph.CalcForces,
		can:      can,
	}
	settings.SubscribeClear()
	settings.SubscribeAdd(w.update)
	return w
}

func (w *Wheel) update(s settings.Settings) error {
	w.mu.Lock()
	defer w.mu.Unlock()
	w.coggingTorqueCancel = s.CoggingTorqueCancel
	w.viscosity = s.Viscosity
	w.softLockForceMagnitude = s.SoftLockForceMagnitude
	HalfLock2Lock := s.Lock2Lock / 2
	MaxAngle := 32768*HalfLock2Lock/360 - 1
	w.fit = utils.Map(-MaxAngle, MaxAngle, -32767, 32767)
	w.limitForce = utils.Limit(-s.MaxCenteringForce, s.MaxCenteringForce)
	motor.SetNeutralAdjust(s.NeutralAdjust)
	return nil
}

func (w *Wheel) SetRStick(x, y int32) {
	w.mu.Lock()
	defer w.mu.Unlock()
	w.rx = limitS16(x)
	w.ry = limitS16(y)
}

func (w *Wheel) Loop(ctx context.Context) error {
	if err := motor.Setup(w.can); err != nil {
		return err
	}
	cnt := 0
	sleep := false
	angle, lastAngle := int32(0), int32(0)
	lastTime := time.Now()
	tick := time.NewTicker(1 * time.Millisecond)
	for {
		select {
		case <-ctx.Done():
			return nil
		case <-tick.C:
			state, err := motor.GetState(w.can)
			if err != nil {
				return err
			}
			w.mu.RLock()
			verocity := 256 * int32(state.Verocity) / 220
			angle = w.fit(state.Angle)
			output := w.limitForce(-angle)          // Centering
			cog := w.coggingTorqueCancel * verocity // Cogging Torque Cancel
			decel := -w.viscosity * pow3(verocity)  // Viscosity
			output += int32(cog + decel)            // Sum
			force := w.calc()
			switch {
			case angle > 32767:
				output -= w.softLockForceMagnitude * (angle - 32767)
			case angle < -32767:
				output -= w.softLockForceMagnitude * (angle + 32767)
			}
			output -= force[0]
			cnt++
			if cnt < 300 {
				output = output * int32(cnt) / 300 // slow start
			}
			v := int16(limitS16(output))
			if sleep {
				v = 0
			}
			rx, ry := w.rx, w.ry
			w.mu.RUnlock()
			if err := motor.Output(w.can, v); err != nil {
				return err
			}
			now := time.Now()
			timeout := now.Sub(lastTime) > 10*time.Second
			d := (angle - lastAngle)
			active := utils.Abs(d) > 40
			wakeup := utils.Abs(d) > 800
			w.mu.Lock()
			if !sleep {
				if active {
					lastTime = now
					lastAngle = angle
				}
				if timeout {
					sleep = true
					//println("enter sleep mode")
					//motor.Disable(w.can)
					lastTime = now
					lastAngle = angle
				}
			} else {
				if wakeup {
					sleep = false
					//println("leave sleep mode")
					//motor.Enable(w.can)
					lastTime = now
					lastAngle = angle
				}
			}
			w.mu.Unlock()
			limitedAngle := int(limitS16(angle))
			w.SetAxis(0, limitedAngle)
			w.SetAxis(2, limitedAngle)
			w.SetAxis(3, int(rx))
			w.SetAxis(4, int(ry))
			if !sleep {
				w.SendState()
			}
		}
	}
}
