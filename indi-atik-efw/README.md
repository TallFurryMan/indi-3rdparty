# Atik EFW1 Serial Transport

This driver talks to the legacy Atik EFW1 through the operating system serial
device created for the wheel's FTDI USB adapter. The wheel protocol remains the
same framed byte protocol used by the previous direct USB transport.

On Linux, the wheel must be recognized by the system as a serial port with an
associated serial device, such as `/dev/ttyUSB0`. Point the INDI serial
connection port to that device; the driver does not open the FTDI USB device
directly.

## Flow Control

The previous libusb implementation configured FTDI RTS/CTS flow control with a
vendor request before sending wheel commands. The serial implementation uses
INDI's standard raw 9600 8N1 serial connection, which leaves hardware flow
control disabled. If a specific host binding requires RTS/CTS for this adapter,
that must be handled by the operator or by a future INDI serial flow-control
extension.
