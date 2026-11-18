import pyads

# Configuration parameters

LINUX_IP = "169.254.100.100" # ipv4 of ethernet port (enx) that connects to PLC
                             # to display ip addresses: linux >> bash >> ip -brief address show
LINUX_NETID = "169.254.100.100.1.1" # add .1.1 to LINUX_IP

PLC_IP = "169.254.184.218" # windows >> twincat ide >> ...
PLC_NETID = "5.155.180.125.1.1" # windows >> twincat ide >> ...

pyads.open_port()

# do this once; check in windows >> twincat >> static routes
#
# route = pyads.add_route_to_plc(
#     sending_net_id=LINUX_NETID,
#     adding_host_name=LINUX_IP,
#     ip_address=PLC_IP,
#     username="Administrator",
#     password="1",
#     route_name="Linux"
# )

plc = pyads.Connection(PLC_NETID, pyads.PORT_TC3PLC1, PLC_IP)
try:
    plc.open()
    
    print(plc.read_by_name("MAIN.bStartHoming"))
    plc.write_by_name("MAIN.bStartHoming", True)
    
except Exception as e:
    print(f"\nException: {e}")
finally:
    print('goodbye gopal')
    plc.close()

pyads.close_port()
