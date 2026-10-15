import pyads
import time

# Configuration parameters

LINUX_IP = "169.254.100.100" # ipv4 of ethernet port that connects to PLC
                             # to display ip addresses: linux >> bash >> ip -brief address show
LINUX_NETID = "169.254.100.100.1.1" # add .1.1 to LINUX_IP

PLC_IP = "169.254.184.218" # windows >> twincat ide >> ...
PLC_NETID = "5.155.180.125.1.1" # windows >> twincat ide >> ...

pyads.open_port()

# do this once; check in windows >> twincat >> static routes
route = pyads.add_route_to_plc(
    sending_net_id=LINUX_NETID,
    adding_host_name=LINUX_IP,
    ip_address=PLC_IP,
    username="Administrator",
    password="1",
    route_name="Linux"
)

# pyads.add_route(PLC_NETID, PLC_IP)
plc = pyads.Connection(PLC_NETID, pyads.PORT_TC3PLC1, PLC_IP)

try:
    plc.open()
    print(f'{plc.read_state()}') # (5, 0) means PLC is in run mode
    print(plc.read_by_name("MAIN.bStartHoming"))
    print(plc.read_by_name("MAIN.bEnableAxes"))

    #plc.write_by_name("MAIN.bResetAxes", True, pyads.PLCTYPE_BOOL)
    #plc.write_by_name("MAIN.bStartHoming", True, pyads.PLCTYPE_BOOL)
    plc.write_by_name("MAIN.bEnableAxes", False, pyads.PLCTYPE_BOOL)
    # plc.write_by_name("MAIN.bEnableAxis", True, pyads.PLCTYPE_BOOL)
    #plc.write_by_name("MAIN.bStartCycle", True, pyads.PLCTYPE_BOOL)

    # plc.write_by_name("MAIN.bSuctionOn", True, pyads.PLCTYPE_BOOL)

    # time.sleep(3)
    # print('i am here')
    # plc.write_by_name("MAIN.bSuctionOn", False, pyads.PLCTYPE_BOOL)
    
    ##plc.write_by_name("MAIN.MotionCtrl.bPowerStatus", True, pyads.PLCTYPE_BOOL)

    # a = plc.read_by_name("MAIN.bEnableAxis")
    # b = plc.read_by_name("MAIN.bStartCycle")
    # c = plc.read_by_name("MAIN.MotionCtrl.bPowerStatus")

    # print(a, b,c)
    
except Exception as e:
    print(f"\nException: {e}")
finally:
    print('goodbye gopal')
    plc.close()

pyads.close_port()
