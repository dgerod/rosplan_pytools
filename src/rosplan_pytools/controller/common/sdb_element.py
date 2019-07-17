
class Element(object):

    @staticmethod
    def extract_ros_type(message):
        return message._type

    def __init__(self, value=None, metadata=""):

        self._metadata = metadata

        if value is not None:
            self._ros_message_value = value
            self._ros_message_type = self.extract_ros_type(value)
        else:
            self._ros_message_value = None
            self._ros_message_type = ""

    def __str__(self):

        if self._ros_message_value is not None:
            value = self._ros_message_value
        else:
            value = ""

        return "%s, %s, %s" % (self._ros_message_type, value, self._metadata)

    def __eq__(self, other):

        return self._ros_message_type == other._ros_message_type

    def clean(self):
        self._metadata = ""
        self._ros_message_value = None
        self._ros_message_type = ""

    def is_valid(self):
        return self._ros_message_value is not None

    def metadata(self):
        return self._metadata

    def value(self):
        return self._ros_message_value

    def type(self):
        return self._ros_message_type

