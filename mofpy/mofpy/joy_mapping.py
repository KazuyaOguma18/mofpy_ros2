from abc import abstractmethod
import textwrap

import six


class JoyMapping(object):
    AXIS = "axis"
    BUTTON = "button"

    def __init__(self, virt_type, name, definition):
        self.name = name
        self.virt_type = virt_type
        self.real_type = definition["real"]["type"]
        self.real_index = definition["real"]["index"]
        self.value = None

    @abstractmethod
    def update_value(self, msg):
        raise NotImplementedError("Method update() not implemented!")


class VirtAxis(JoyMapping):
    def __init__(self, name, definition):
        super(VirtAxis, self).__init__(JoyMapping.AXIS, name, definition)
        self._is_ps_shoulder = False
        self._button_counterpart = None

        if isinstance(definition, six.string_types):
            return

        # See docstring in __not_pressed_yet__
        self._is_ps_shoulder = "ps_shoulder_counterpart" in definition
        if self._is_ps_shoulder:
            self._button_counterpart = definition["ps_shoulder_counterpart"]

        self._range = definition["range"] if "range" in definition else None
        r = self._range
        if r and r[0] > r[1]:
            r[0], r[1] = r[1], r[0]

    def update_value(self, msg):
        # Normal case: treat axis as axis
        if self.real_type == JoyMapping.AXIS:
            arr = msg.axes
            if self._is_ps_shoulder:
                if self.__not_pressed_yet__(msg):
                    v = 0
                else:
                    # 1.0 when not pressed, -1.0 when fully pressed
                    v = (-arr[self.real_index] + 1) * 0.5
            else:
                v = arr[self.real_index]

            if self._range:
                if v < self._range[0]:
                    v = self._range[0]
                elif v > self._range[1]:
                    v = self._range[1]
            self.value = v
        # Special case: treat button as axis
        else:
            self.value = msg.buttons[self.real_index]

        return self.value

    def __not_pressed_yet__(self, msg):
        """
        Map raw joystick inputs to named virtual axes and buttons.

        This class retrieves predefined virtual input definitions, processes
        joystick messages, and provides a structured dictionary containing
        named axes and buttons.

        :param msg:
        :return: True if the axis hasn't been pressed yet
        """
        if not self._is_ps_shoulder:
            return False
        if self._button_counterpart is None:
            return False

        return msg.axes[self.real_index] == 0 and msg.buttons[self._button_counterpart] == 0

    def __str__(self):
        s = """\
        Virtual Axis {0}:
          from:           {1} {2}
          is_ps_shoulder: {3}
          counterpart:    {4}\
          """.format(
            self.name,
            self.real_type,
            self.real_index,
            self._is_ps_shoulder,
            self._button_counterpart,
        )
        return textwrap.dedent(s)


class VirtButton(JoyMapping):
    """
    True when pressed (active), False otherwise (inactive).

    :type value: bool
    """

    def __init__(self, name, definition):
        super(VirtButton, self).__init__(JoyMapping.BUTTON, name, definition)
        self._is_active_low = False
        if "is_active_low" in definition:
            self._is_active_low = definition["is_active_low"]

        # Make a button from an axis
        if self.real_type == JoyMapping.AXIS:
            self._low_range = definition["low_range"]
            lr = self._low_range
            if lr[0] > lr[1]:
                lr[0], lr[1] = lr[1], lr[0]

            self._high_range = definition["high_range"]
            hr = self._high_range
            if hr[0] > hr[1]:
                hr[0], hr[1] = hr[1], hr[0]

    def update_value(self, msg):
        # Normal case: treat button as button
        if self.real_type == JoyMapping.BUTTON:
            self.value = msg.buttons[self.real_index] == 1

            if self._is_active_low:
                self.value = not self.value
        # Special case: treat axis as button
        else:
            v = msg.axes[self.real_index]
            if self._low_range[0] <= v <= self._low_range[1]:
                self.value = False
            elif self._high_range[0] <= v <= self._high_range[1]:
                self.value = True
            else:
                # Not in range: default to low
                self.value = False

        return self.value

    def __str__(self):
        s = """\
        Virtual Button {0}:
          from:          {1} {2}
          is_active_low: {3}
          low_range:     {4}
          high_range:    {5}\
          """.format(
            self.name,
            self.real_type,
            self.real_index,
            self._is_active_low,
            self._low_range,
            self._high_range,
        )
        return textwrap.dedent(s)
