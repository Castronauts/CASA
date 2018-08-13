import xml
import KeyboardController
import time

control_spec = xml.etree.ElementTree.parse('cartesian.xml')
keyboard_spec = xml.etree.ElementTree.parse('cartesian_keyboard.xml')

kc = KeyboardController.DiscreteKeyboardController(control_spec, keyboard_spec)
kc.start_listener()
while True:
	print(kc.get_all_controls())
	time.sleep(1)