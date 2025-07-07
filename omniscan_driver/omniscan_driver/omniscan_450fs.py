from omni_resource import Device4Omniscan

def main():
    import argparse


    parser = argparse.ArgumentParser(description="Ping python library example.")
    parser.add_argument('--device', action="store", required=False, type=str, help="Ping device port. E.g: /dev/ttyUSB0")
    parser.add_argument('--baudrate', action="store", type=int, default=115200, help="Ping device baudrate. E.g: 115200")
    parser.add_argument('--udp', action="store", required=False, type=str, help="Ping UDP server. E.g: 0.0.0.0:12345")
    parser.add_argument('--tcp', action="store", required=False, type=str, help="Ping TCP server. E.g: 127.0.0.1:12345")
    args = parser.parse_args()
    if args.device is None and args.udp is None and args.tcp is None:
        parser.print_help()
        exit(1)

    p = Device4Omniscan()

    if args.device is not None:
        p.connect_serial(args.device, args.baudrate)
    elif args.udp is not None:
        (host, port) = args.udp.split(':')
        p.connect_udp(host, int(port))
    elif args.tcp is not None:
        (host, port) = args.tcp.split(':')
        p.connect_tcp(host, int(port))


    print("Initialized: %s" % p.initialize())
    
    print("\ntesting get_protocol_version")
    result = p.get_protocol_version()
    print("  " + str(result))
    print("  > > pass: %s < <" % (result is not None))
    
    print("\ntesting get_device_information")
    result = p.get_device_information()
    print("  " + str(result))
    print("  > > pass: %s < <" % (result is not None))

if __name__ == "__main__":
    main()