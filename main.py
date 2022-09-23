from serial import Serial
from threading import Thread, Event

LIST_OF_COMMANDS = ('blink', 'on', 'off', 'hz=%d')
EXIT_CMD = 'exit'
MIN_HZ = 0
MAX_HZ = 10


def is_hz_command(cmd: str):
    return cmd.startswith('hz=')


def is_valid_hz_value(cmd: str):
    _, value = cmd.split('=')
    if not value.isdigit():
        return False
    parsed_value = int(value)
    return MIN_HZ <= parsed_value <= MAX_HZ


def is_valid_command(cmd: str):
    if not cmd or cmd == '\n':
        return False
    if not (cmd in LIST_OF_COMMANDS or is_hz_command(cmd)):
        print('Wrong command')
        return False
    if is_hz_command(cmd) and not is_valid_hz_value(cmd):
        print('Wrong hz value')
        print(f'Valid value could be between {MIN_HZ} and {MAX_HZ}')
        return False
    return True


def listener(serial: Serial, event_done: Event):
    while not event_done.is_set():
        value = serial.readline()
        if not value:
            continue
        print(f'From controller: {value.decode().strip()}')


def main():
    print(f'List of commands: {LIST_OF_COMMANDS}')
    listener_event_done = Event()
    with Serial('COM4', baudrate=115200, timeout=3) as serial:
        listener_thread = Thread(
            target=listener,
            args=(serial, listener_event_done),
            daemon=True,
        )
        listener_thread.start()
        while True:
            cmd = input('Command: ').strip()
            if cmd == EXIT_CMD:
                break
            if not is_valid_command(cmd):
                continue
            serial.write(cmd.encode())
        listener_event_done.set()


if __name__ == '__main__':
    main()
