""" MECHA 15 - Harry """
"""Keyboard-commanded, open-loop drive task for the Romi.

The task receives a persistent command through ``drive_command`` and applies
the corresponding effort directly to both motor drivers.  It intentionally
does not use the closed-loop test tasks: those tests collect a fixed-length
data set and would stop the robot after 101 samples.
"""


class KeyboardVCP:
    """Pass-through USB VCP wrapper which adds a manual-drive mode.

    The first W enters manual mode without moving. While manual mode is active,
    W/A/S/D/X/space are intercepted and returned as NUL bytes so the existing
    TaskUser menu safely ignores them.
    """

    _COMMANDS = {
        ord("s"): "reverse",
        ord("S"): "reverse",
        ord("a"): "left",
        ord("A"): "left",
        ord("d"): "right",
        ord("D"): "right",
        ord("x"): "stop",
        ord("X"): "stop",
        ord(" "): "stop",
    }

    def __init__(self, vcp, drive_command, manual_go):
        self._vcp = vcp
        self._drive_command = drive_command
        self._manual_go = manual_go

    def any(self):
        return self._vcp.any()

    def read(self, nbytes=None):
        data = self._vcp.read() if nbytes is None else self._vcp.read(nbytes)
        if data is None:
            return data

        filtered = bytearray(data)
        for index, byte in enumerate(filtered):
            if byte in (ord("w"), ord("W")):
                if self._manual_go.value:
                    self._drive_command.value = "forward"
                else:
                    self._manual_go.value = True
                    self._drive_command.value = "stop"
                    self._vcp.write("\r\nManual control enabled. W=forward, S=reverse, A=left, D=right, X/space=stop\r\n")
                filtered[index] = 0
                continue

            command = self._COMMANDS.get(byte)
            if command is not None and self._manual_go.value:
                self._drive_command.value = command
                filtered[index] = 0

        return bytes(filtered)

    def write(self, data):
        return self._vcp.write(data)


class TaskWASD:
    """Continuously apply the current manual-drive command to the motors."""

    def __init__(self, left_mot, right_mot, drive_command, manual_go, l_go,
                 r_go, line_go, man_go, effort=35):
        self.left_mot = left_mot
        self.right_mot = right_mot
        self.drive_command = drive_command
        self.manual_go = manual_go
        self.l_go = l_go
        self.r_go = r_go
        self.line_go = line_go
        self.man_go = man_go
        self.effort = effort
        self.last_command = None

    def _stop(self):
        self.left_mot.set_effort(0)
        self.right_mot.set_effort(0)
        self.left_mot.disable()
        self.right_mot.disable()

    def _cancel_other_motion(self):
        """Prevent another task from continuing a test or line-following run."""
        self.l_go.value = False
        self.r_go.value = False
        self.line_go.value = False
        self.man_go.value = False

    def run(self):
        self._stop()

        while True:
            command = self.drive_command.value

            if not self.manual_go.value or command == "stop":
                self._stop()
            else:
                self._cancel_other_motion()
                self.left_mot.enable()
                self.right_mot.enable()

                if command == "forward":
                    self.left_mot.set_effort(self.effort)
                    self.right_mot.set_effort(self.effort)
                elif command == "reverse":
                    self.left_mot.set_effort(-self.effort)
                    self.right_mot.set_effort(-self.effort)
                elif command == "left":
                    self.left_mot.set_effort(-self.effort)
                    self.right_mot.set_effort(self.effort)
                elif command == "right":
                    self.left_mot.set_effort(self.effort)
                    self.right_mot.set_effort(-self.effort)
                else:
                    self.drive_command.value = "stop"
                    self._stop()

            self.last_command = command
            yield 0
