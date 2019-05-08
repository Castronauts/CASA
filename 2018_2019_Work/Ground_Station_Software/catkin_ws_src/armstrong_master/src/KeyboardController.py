from InputController import InputController
import pynput
import threading


"""
Class for creating keyboard controllers that use discrete (pressed vs unpressed) control signals
"""
class DiscreteKeyboardController(InputController):

	def __init__(self, controller_spec, keyboard_layout):
		InputController.__init__(self, controller_spec)
		self.setup_key_listeners(keyboard_layout)

	def setup_key_listeners(self, keyboard_layout):
		self.controls_map = {}
		for input_signal in keyboard_layout.iter('key'):
			input_key = input_signal.attrib['name']
			input_name = input_signal.find('input_name').text
			pressed_value = input_signal.find('pressed_value').text
			unpressed_value = input_signal.find('unpressed_value').text
			self.controls_map[str(input_key)] = {}
			self.controls_map[input_key]['name'] = input_name
			self.controls_map[input_key]['pressed'] = pressed_value
			self.controls_map[input_key]['unpressed'] = unpressed_value

	def on_press(self, key):
		try:
			key_pressed = key.char
		except:
			return
		if key_pressed in self.controls_map:
			key_spec = self.controls_map[key_pressed]
			self.update_control_signal(key_spec['name'], key_spec['pressed'])

	def on_release(self, key):
		try:
			key_pressed = key.char
		except:
			return
		if key_pressed in self.controls_map:
			key_spec = self.controls_map[key_pressed]
			self.update_control_signal(key_spec['name'], key_spec['unpressed'])


	def start_listener(self):
		self.listener = pynput.keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
		self.listener.start()
		threading.Thread(target=self._listen).start()

	def _listen(self):
		self.listener.join()

	def stop_listener(self):
		self.listener.stop()