from dronekit import connect

vehicle = connect("127.0.0.1:14551", wait_ready=True)

print('connected')

print('armed state: {}'.format(vehicle.armed))