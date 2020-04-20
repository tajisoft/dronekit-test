from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

vehicle = connect('com9,115200', wait_ready=True)

print('Vehicle connected')

print(
    'Vehicle armed state: {}'.format(vehicle.armed)
)

vehicle.armed = True

time.sleep(3)

print(
    'Vehicle armed state: {}'.format(vehicle.armed)
)

print(vehicle.location)
print(vehicle.location.global_frame)
print(vehicle.location.global_relative_frame)

def global_frame_callback(self, name, val):
    print('callback')
    print('{} {} {}'.format(self, name, val))
    print('{} {} {}'.format(type(self), type(name), type(val)))

vehicle.add_attribute_listener('location.global_relative_frame', global_frame_callback)

time.sleep(30)