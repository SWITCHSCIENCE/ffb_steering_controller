package pid

//go:align 4
var Descriptor = []byte{
	0x05, 0x01, // USAGE_PAGE (Generic Desktop)
	0x09, 0x04, // USAGE (0x04:Joystick/0x05:Gamepad/0x08:Multi-axis)
	0xa1, 0x01, // COLLECTION (Application)
	0x09, 0x01, // USAGE (Pointer)
	0x85, 0x01, // REPORT_ID (1)
	0xa1, 0x00, // COLLECTION (Physical) #1

	// buttons
	0x05, 0x09, // USAGE_PAGE  (Button)
	0x19, 0x01, // USAGE_MINIMUM (Button 1)
	0x29, 0x18, // USAGE_MAXIMUM (Button 24)
	0x15, 0x00, // LOGICAL_MINIMUM (0)
	0x25, 0x01, // LOGICAL_MAXIMUM (1)
	0x75, 0x01, // REPORT_SIZE (1)
	0x95, 0x18, // REPORT_COUNT (24)
	0x55, 0x00, // Unit Exponent (-16)
	0x65, 0x00, // Unit (0x00)
	0x81, 0x02, // INPUT (Data/Var/Abs)

	// x
	0x05, 0x01, // USAGE_PAGE (Generic Desktop)
	0x09, 0x30, // USAGE (X)
	0x16, 0x01, 0x80, // LOGICAL_MINIMUM (-32767)
	0x26, 0xff, 0x7f, // LOGICAL_MAXIMUM (32767)
	0x75, 0x10, // REPORT_SIZE (16)
	0x95, 0x01, // REPORT_COUNT (1)
	0x81, 0x02, // INPUT (Data/Var/Abs)
	// z
	0x05, 0x01, // USAGE_PAGE (Generic Desktop)
	0x09, 0x32, // USAGE (Z)
	0x16, 0x00, 0x00, // LOGICAL_MINIMUM (0)
	0x26, 0xff, 0x7f, // LOGICAL_MAXIMUM (32767)
	0x75, 0x10, // REPORT_SIZE (16)
	0x95, 0x01, // REPORT_COUNT (1)
	0x81, 0x02, // INPUT (Data/Var/Abs)
	// throttle clutch brake steering
	0x05, 0x02, // USAGE_PAGE (Simulation Controls)
	0x09, 0xbb, // USAGE (Throttle)
	0x09, 0xc4, // USAGE (Accelerator)
	0x09, 0xc5, // USAGE (Brake)
	0x16, 0x00, 0x00, // LOGICAL_MINIMUM (0)
	0x26, 0xff, 0x7f, // LOGICAL_MAXIMUM (32767)
	0x75, 0x10, // REPORT_SIZE (16)
	0x95, 0x03, // REPORT_COUNT (3)
	0x81, 0x02, // INPUT (Data/Var/Abs)
	// steering
	0x05, 0x02, // USAGE_PAGE (Simulation Controls)
	0x09, 0xc8, // USAGE (Steering)
	0x16, 0x01, 0x80, // LOGICAL_MINIMUM (-32767)
	0x26, 0xff, 0x7f, // LOGICAL_MAXIMUM (32767)
	0x75, 0x10, // REPORT_SIZE (16)
	0x95, 0x01, // REPORT_COUNT (1)
	0x81, 0x02, // INPUT (Data/Var/Abs)

	// end collection
	0xc0, // END_COLLECTION #1

	// Physical Interface Device Definition
	0x05, 0x0f, 0x09, 0x92, 0xa1, 0x02, 0x85,
	0x02, 0x09, 0x9f, 0x09, 0xa0, 0x09, 0xa4, 0x09,
	0xa5, 0x09, 0xa6, 0x15, 0x00, 0x25, 0x01, 0x35,
	0x00, 0x45, 0x01, 0x75, 0x01, 0x95, 0x05, 0x81,
	0x02, 0x95, 0x03, 0x81, 0x03, 0x09, 0x94, 0x15,
	0x00, 0x25, 0x01, 0x35, 0x00, 0x45, 0x01, 0x75,
	0x01, 0x95, 0x01, 0x81, 0x02, 0x09, 0x22, 0x15,
	0x01, 0x25, 0x28, 0x35, 0x01, 0x45, 0x28, 0x75,
	0x07, 0x95, 0x01, 0x81, 0x02, 0xc0, 0x09, 0x21,
	0xa1, 0x02, 0x85, 0x01, 0x09, 0x22, 0x15, 0x01,
	0x25, 0x28, 0x35, 0x01, 0x45, 0x28, 0x75, 0x08,
	0x95, 0x01, 0x91, 0x02, 0x09, 0x25, 0xa1, 0x02,
	0x09, 0x26, 0x09, 0x27, 0x09, 0x30, 0x09, 0x31,
	0x09, 0x32, 0x09, 0x33, 0x09, 0x34, 0x09, 0x40,
	0x09, 0x41, 0x09, 0x42, 0x09, 0x43, 0x09, 0x28,
	0x09, 0x28, 0x15, 0x01, 0x25, 0x0c, 0x35, 0x01,
	0x45, 0x0c, 0x75, 0x08, 0x95, 0x01, 0x91, 0x00,
	0xc0, 0x09, 0x50, 0x09, 0x54, 0x09, 0x51, 0x15,
	0x00, 0x26, 0xff, 0x7f, 0x35, 0x00, 0x46, 0xff,
	0x7f, 0x66, 0x03, 0x10, 0x55, 0xfd, 0x75, 0x10,
	0x95, 0x03, 0x91, 0x02, 0x55, 0x00, 0x66, 0x00,
	0x00, 0x09, 0x52, 0x15, 0x00, 0x26, 0xff, 0x00,
	0x35, 0x00, 0x46, 0x10, 0x27, 0x75, 0x08, 0x95,
	0x01, 0x91, 0x02, 0x09, 0x53, 0x15, 0x01, 0x25,
	0x08, 0x35, 0x01, 0x45, 0x08, 0x75, 0x08, 0x95,
	0x01, 0x91, 0x02, 0x09, 0x55, 0xa1, 0x02, 0x05,
	0x01, 0x09, 0x30, 0x09, 0x31, 0x15, 0x00, 0x25,
	0x01, 0x75, 0x01, 0x95, 0x02, 0x91, 0x02, 0xc0,
	0x05, 0x0f, 0x09, 0x56, 0x95, 0x01, 0x91, 0x02,
	0x95, 0x05, 0x91, 0x03, 0x09, 0x57, 0xa1, 0x02,
	0x0b, 0x01, 0x00, 0x0a, 0x00, 0x0b, 0x02, 0x00,
	0x0a, 0x00, 0x66, 0x14, 0x00, 0x55, 0xfe, 0x15,
	0x00, 0x26, 0xff, 0x00, 0x35, 0x00, 0x47, 0xa0,
	0x8c, 0x00, 0x00, 0x66, 0x00, 0x00, 0x75, 0x08,
	0x95, 0x02, 0x91, 0x02, 0x55, 0x00, 0x66, 0x00,
	0x00, 0xc0, 0x05, 0x0f, 0x09, 0x58, 0xa1, 0x02,
	0x0b, 0x01, 0x00, 0x0a, 0x00, 0x0b, 0x02, 0x00,
	0x0a, 0x00, 0x26, 0xfd, 0x7f, 0x75, 0x10, 0x95,
	0x02, 0x91, 0x02, 0xc0, 0xc0, 0x09, 0x5a, 0xa1,
	0x02, 0x85, 0x02, 0x09, 0x22, 0x15, 0x01, 0x25,
	0x28, 0x35, 0x01, 0x45, 0x28, 0x75, 0x08, 0x95,
	0x01, 0x91, 0x02, 0x09, 0x5b, 0x09, 0x5d, 0x16,
	0x00, 0x00, 0x26, 0x10, 0x27, 0x36, 0x00, 0x00,
	0x46, 0x10, 0x27, 0x75, 0x10, 0x95, 0x02, 0x91,
	0x02, 0x09, 0x5c, 0x09, 0x5e, 0x66, 0x03, 0x10,
	0x55, 0xfd, 0x27, 0xff, 0x7f, 0x00, 0x00, 0x47,
	0xff, 0x7f, 0x00, 0x00, 0x75, 0x20, 0x95, 0x02,
	0x91, 0x02, 0x45, 0x00, 0x66, 0x00, 0x00, 0x55,
	0x00, 0xc0, 0x09, 0x5f, 0xa1, 0x02, 0x85, 0x03,
	0x09, 0x22, 0x15, 0x01, 0x25, 0x28, 0x35, 0x01,
	0x45, 0x28, 0x75, 0x08, 0x95, 0x01, 0x91, 0x02,
	0x09, 0x23, 0x15, 0x00, 0x25, 0x03, 0x35, 0x00,
	0x45, 0x03, 0x75, 0x04, 0x95, 0x01, 0x91, 0x02,
	0x09, 0x58, 0xa1, 0x02, 0x0b, 0x01, 0x00, 0x0a,
	0x00, 0x0b, 0x02, 0x00, 0x0a, 0x00, 0x75, 0x02,
	0x95, 0x02, 0x91, 0x02, 0xc0, 0x16, 0xf0, 0xd8,
	0x26, 0x10, 0x27, 0x36, 0xf0, 0xd8, 0x46, 0x10,
	0x27, 0x09, 0x60, 0x75, 0x10, 0x95, 0x01, 0x91,
	0x02, 0x36, 0xf0, 0xd8, 0x46, 0x10, 0x27, 0x09,
	0x61, 0x09, 0x62, 0x95, 0x02, 0x91, 0x02, 0x16,
	0x00, 0x00, 0x26, 0x10, 0x27, 0x36, 0x00, 0x00,
	0x46, 0x10, 0x27, 0x09, 0x63, 0x09, 0x64, 0x75,
	0x10, 0x95, 0x02, 0x91, 0x02, 0x09, 0x65, 0x46,
	0x10, 0x27, 0x95, 0x01, 0x91, 0x02, 0xc0, 0x09,
	0x6e, 0xa1, 0x02, 0x85, 0x04, 0x09, 0x22, 0x15,
	0x01, 0x25, 0x28, 0x35, 0x01, 0x45, 0x28, 0x75,
	0x08, 0x95, 0x01, 0x91, 0x02, 0x09, 0x70, 0x16,
	0x00, 0x00, 0x26, 0x10, 0x27, 0x36, 0x00, 0x00,
	0x46, 0x10, 0x27, 0x75, 0x10, 0x95, 0x01, 0x91,
	0x02, 0x09, 0x6f, 0x16, 0xf0, 0xd8, 0x26, 0x10,
	0x27, 0x36, 0xf0, 0xd8, 0x46, 0x10, 0x27, 0x95,
	0x01, 0x75, 0x10, 0x91, 0x02, 0x09, 0x71, 0x66,
	0x14, 0x00, 0x55, 0xfe, 0x15, 0x00, 0x27, 0x9f,
	0x8c, 0x00, 0x00, 0x35, 0x00, 0x47, 0x9f, 0x8c,
	0x00, 0x00, 0x75, 0x10, 0x95, 0x01, 0x91, 0x02,
	0x09, 0x72, 0x15, 0x00, 0x27, 0xff, 0x7f, 0x00,
	0x00, 0x35, 0x00, 0x47, 0xff, 0x7f, 0x00, 0x00,
	0x66, 0x03, 0x10, 0x55, 0xfd, 0x75, 0x20, 0x95,
	0x01, 0x91, 0x02, 0x66, 0x00, 0x00, 0x55, 0x00,
	0xc0, 0x09, 0x73, 0xa1, 0x02, 0x85, 0x05, 0x09,
	0x22, 0x15, 0x01, 0x25, 0x28, 0x35, 0x01, 0x45,
	0x28, 0x75, 0x08, 0x95, 0x01, 0x91, 0x02, 0x09,
	0x70, 0x16, 0xf0, 0xd8, 0x26, 0x10, 0x27, 0x36,
	0xf0, 0xd8, 0x46, 0x10, 0x27, 0x75, 0x10, 0x95,
	0x01, 0x91, 0x02, 0xc0, 0x09, 0x74, 0xa1, 0x02,
	0x85, 0x06, 0x09, 0x22, 0x15, 0x01, 0x25, 0x28,
	0x35, 0x01, 0x45, 0x28, 0x75, 0x08, 0x95, 0x01,
	0x91, 0x02, 0x09, 0x75, 0x09, 0x76, 0x16, 0xf0,
	0xd8, 0x26, 0x10, 0x27, 0x36, 0xf0, 0xd8, 0x46,
	0x10, 0x27, 0x75, 0x10, 0x95, 0x02, 0x91, 0x02,
	0xc0, 0x09, 0x68, 0xa1, 0x02, 0x85, 0x07, 0x09,
	0x22, 0x15, 0x01, 0x25, 0x28, 0x35, 0x01, 0x45,
	0x28, 0x75, 0x08, 0x95, 0x01, 0x91, 0x02, 0x09,
	0x6c, 0x15, 0x00, 0x26, 0x10, 0x27, 0x35, 0x00,
	0x46, 0x10, 0x27, 0x75, 0x10, 0x95, 0x01, 0x91,
	0x02, 0x09, 0x69, 0x15, 0x81, 0x25, 0x7f, 0x35,
	0x00, 0x46, 0xff, 0x00, 0x75, 0x08, 0x95, 0x0c,
	0x92, 0x02, 0x01, 0xc0, 0x09, 0x66, 0xa1, 0x02,
	0x85, 0x08, 0x05, 0x01, 0x09, 0x30, 0x09, 0x31,
	0x15, 0x81, 0x25, 0x7f, 0x35, 0x00, 0x46, 0xff,
	0x00, 0x75, 0x08, 0x95, 0x02, 0x91, 0x02, 0xc0,
	0x05, 0x0f, 0x09, 0x77, 0xa1, 0x02, 0x85, 0x0a,
	0x09, 0x22, 0x15, 0x01, 0x25, 0x28, 0x35, 0x01,
	0x45, 0x28, 0x75, 0x08, 0x95, 0x01, 0x91, 0x02,
	0x09, 0x78, 0xa1, 0x02, 0x09, 0x79, 0x09, 0x7a,
	0x09, 0x7b, 0x15, 0x01, 0x25, 0x03, 0x75, 0x08,
	0x95, 0x01, 0x91, 0x00, 0xc0, 0x09, 0x7c, 0x15,
	0x00, 0x26, 0xff, 0x00, 0x35, 0x00, 0x46, 0xff,
	0x00, 0x91, 0x02, 0xc0, 0x09, 0x90, 0xa1, 0x02,
	0x85, 0x0b, 0x09, 0x22, 0x15, 0x01, 0x25, 0x28,
	0x35, 0x01, 0x45, 0x28, 0x75, 0x08, 0x95, 0x01,
	0x91, 0x02, 0xc0, 0x09, 0x96, 0xa1, 0x02, 0x85,
	0x0c, 0x09, 0x97, 0x09, 0x98, 0x09, 0x99, 0x09,
	0x9a, 0x09, 0x9b, 0x09, 0x9c, 0x15, 0x01, 0x25,
	0x06, 0x75, 0x08, 0x95, 0x01, 0x91, 0x00, 0xc0,
	0x09, 0x7d, 0xa1, 0x02, 0x85, 0x0d, 0x09, 0x7e,
	0x15, 0x00, 0x26, 0xff, 0x00, 0x35, 0x00, 0x46,
	0x10, 0x27, 0x75, 0x08, 0x95, 0x01, 0x91, 0x02,
	0xc0, 0x09, 0x6b, 0xa1, 0x02, 0x85, 0x0e, 0x09,
	0x22, 0x15, 0x01, 0x25, 0x28, 0x35, 0x01, 0x45,
	0x28, 0x75, 0x08, 0x95, 0x01, 0x91, 0x02, 0x09,
	0x6d, 0x15, 0x00, 0x26, 0xff, 0x00, 0x35, 0x00,
	0x46, 0xff, 0x00, 0x75, 0x08, 0x95, 0x01, 0x91,
	0x02, 0x09, 0x51, 0x66, 0x03, 0x10, 0x55, 0xfd,
	0x15, 0x00, 0x26, 0xff, 0x7f, 0x35, 0x00, 0x46,
	0xff, 0x7f, 0x75, 0x10, 0x95, 0x01, 0x91, 0x02,
	0x55, 0x00, 0x66, 0x00, 0x00, 0xc0, 0x09, 0xab,
	0xa1, 0x02, 0x85, 0x05, 0x09, 0x25, 0xa1, 0x02,
	0x09, 0x26, 0x09, 0x27, 0x09, 0x30, 0x09, 0x31,
	0x09, 0x32, 0x09, 0x33, 0x09, 0x34, 0x09, 0x40,
	0x09, 0x41, 0x09, 0x42, 0x09, 0x43, 0x09, 0x28,
	0x25, 0x0c, 0x15, 0x01, 0x35, 0x01, 0x45, 0x0c,
	0x75, 0x08, 0x95, 0x01, 0xb1, 0x00, 0xc0, 0x05,
	0x01, 0x09, 0x3b, 0x15, 0x00, 0x26, 0xff, 0x01,
	0x35, 0x00, 0x46, 0xff, 0x01, 0x75, 0x0a, 0x95,
	0x01, 0xb1, 0x02, 0x75, 0x06, 0xb1, 0x01, 0xc0,
	0x05, 0x0f, 0x09, 0x89, 0xa1, 0x02, 0x85, 0x06,
	0x09, 0x22, 0x25, 0x28, 0x15, 0x01, 0x35, 0x01,
	0x45, 0x28, 0x75, 0x08, 0x95, 0x01, 0xb1, 0x02,
	0x09, 0x8b, 0xa1, 0x02, 0x09, 0x8c, 0x09, 0x8d,
	0x09, 0x8e, 0x25, 0x03, 0x15, 0x01, 0x35, 0x01,
	0x45, 0x03, 0x75, 0x08, 0x95, 0x01, 0xb1, 0x00,
	0xc0, 0x09, 0xac, 0x15, 0x00, 0x27, 0xff, 0xff,
	0x00, 0x00, 0x35, 0x00, 0x47, 0xff, 0xff, 0x00,
	0x00, 0x75, 0x10, 0x95, 0x01, 0xb1, 0x00, 0xc0,
	0x09, 0x7f, 0xa1, 0x02, 0x85, 0x07, 0x09, 0x80,
	0x75, 0x10, 0x95, 0x01, 0x15, 0x00, 0x35, 0x00,
	0x27, 0xff, 0xff, 0x00, 0x00, 0x47, 0xff, 0xff,
	0x00, 0x00, 0xb1, 0x02, 0x09, 0x83, 0x26, 0xff,
	0x00, 0x46, 0xff, 0x00, 0x75, 0x08, 0x95, 0x01,
	0xb1, 0x02, 0x09, 0xa9, 0x09, 0xaa, 0x75, 0x01,
	0x95, 0x02, 0x15, 0x00, 0x25, 0x01, 0x35, 0x00,
	0x45, 0x01, 0xb1, 0x02, 0x75, 0x06, 0x95, 0x01,
	0xb1, 0x03, 0xc0, 0xc0,
}
