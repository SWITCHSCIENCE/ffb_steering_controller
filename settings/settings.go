package settings

import (
	"bytes"
	"crypto/sha256"
	"encoding/binary"
	"fmt"
	"io"
)

type Settings struct {
	NeutralAdjust          float32 // unit:deg
	Lock2Lock              int32   // unit:deg
	CoggingTorqueCancel    int32   // 32768 // unit:100*n/256 %
	Viscosity              int32   // 30000 // unit:100*n/256 %
	MaxCenteringForce      int32   // unit:100*n/32767 %
	SoftLockForceMagnitude int32   // unit:100*n %
}

func Marshal(s Settings) []byte {
	buf := bytes.NewBuffer(nil)
	binary.Write(buf, binary.LittleEndian, s.NeutralAdjust)
	binary.Write(buf, binary.LittleEndian, s.Lock2Lock)
	binary.Write(buf, binary.LittleEndian, s.CoggingTorqueCancel)
	binary.Write(buf, binary.LittleEndian, s.Viscosity)
	binary.Write(buf, binary.LittleEndian, s.MaxCenteringForce)
	binary.Write(buf, binary.LittleEndian, s.SoftLockForceMagnitude)
	hash := sha256.Sum256(buf.Bytes())
	buf.Write(hash[:])
	return buf.Bytes()
}

func Unmarshal(b []byte) (Settings, error) {
	check := bytes.NewBuffer(nil)
	buf := bytes.NewBuffer(b)
	tee := io.TeeReader(buf, check)
	var s Settings
	binary.Read(tee, binary.LittleEndian, &s.NeutralAdjust)
	binary.Read(tee, binary.LittleEndian, &s.Lock2Lock)
	binary.Read(tee, binary.LittleEndian, &s.CoggingTorqueCancel)
	binary.Read(tee, binary.LittleEndian, &s.Viscosity)
	binary.Read(tee, binary.LittleEndian, &s.MaxCenteringForce)
	binary.Read(tee, binary.LittleEndian, &s.SoftLockForceMagnitude)
	calculated := sha256.Sum256(check.Bytes())
	hash := make([]byte, 32)
	tee.Read(hash)
	if !bytes.Equal(hash, calculated[:]) {
		return s, fmt.Errorf("invalid hash")
	}
	return s, nil
}

var (
	defaultSettings = Settings{
		NeutralAdjust:          -6.5, // unit:deg   -180.0...180.0
		Lock2Lock:              540,  // unit:deg   180..1440
		CoggingTorqueCancel:    128,  // unit:100*n/256 %  0..255
		Viscosity:              128,  // unit:100*n/256 %  0..255
		MaxCenteringForce:      0,    // unit:100*n/32767 %  0..2048
		SoftLockForceMagnitude: 8,    // unit:100*n %  0..15
	}
	currentSettings = defaultSettings
	subscribe       []func(s Settings) error
)

func Validate(s Settings) error {
	if s.NeutralAdjust < -180 || s.NeutralAdjust > +180 {
		return fmt.Errorf("invalid neutral adjust: %f", s.NeutralAdjust)
	}
	if s.Lock2Lock < 180 || s.Lock2Lock > 1440 {
		return fmt.Errorf("invalid lock to lock: %d", s.Lock2Lock)
	}
	if s.CoggingTorqueCancel < 0 || s.CoggingTorqueCancel > 256 {
		return fmt.Errorf("invalid cogging torque cancel: %d", s.CoggingTorqueCancel)
	}
	if s.Viscosity < 0 || s.Viscosity > 1024 {
		return fmt.Errorf("invalid viscosity: %d", s.Viscosity)
	}
	if s.MaxCenteringForce < 0 || s.MaxCenteringForce > 2048 {
		return fmt.Errorf("invalid max centering force: %d", s.MaxCenteringForce)
	}
	if s.SoftLockForceMagnitude < 0 || s.SoftLockForceMagnitude > 16 {
		return fmt.Errorf("invalid soft lock force magnitude: %d", s.SoftLockForceMagnitude)
	}
	return nil
}

func SubscribeClear() {
	subscribe = nil
}

func SubscribeAdd(f func(s Settings) error) {
	subscribe = append(subscribe, f)
}

func Default() Settings {
	return defaultSettings
}

// Update
func Update(s Settings) error {
	if err := Validate(s); err != nil {
		return err
	}
	// notify all subscribers
	for _, l := range subscribe {
		if err := l(s); err != nil {
			return err
		}
	}
	currentSettings = s
	return nil
}

func Get() Settings {
	return currentSettings
}
