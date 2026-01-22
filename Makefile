.PHONY: run check

run:
	cargo run

check:
	cargo check

read:
	stty -F /dev/ttyACM0 115200 raw -echo
	cat /dev/ttyACM0