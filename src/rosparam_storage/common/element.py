
from rosparam_storage.common.element_converter import data_to_element, element_to_data


class StorageElement(object):

    @staticmethod
    def extract_ros_type(ros_message):
        return ros_message._type

    def __init__(self, value=None, metadata=""):

        self._metadata = metadata

        if value is not None:
            self._ros_message = value
            self._is_valid = True

        else:
            self._ros_message = None
            self._is_valid = False

    def __str__(self):

        if self._ros_message is not None:
            info = "%s, %s, %s" % (self.type(), self._ros_message, self._metadata)
        else:
            info = ""

        return info

    def __eq__(self, other):

        return self.type() == other.type()

    @staticmethod
    def to_string(element):
        # type: (StorageElement) -> (str, str)
        return element_to_data(element)


    @staticmethod
    def from_string(metadata, value):
        # type: (dict) -> (id, StorageElement)
        return data_to_element(metadata, value)

    def clean(self):
        self._metadata = ""
        self._ros_message = None
        self._is_valid = False

    def is_valid(self):
        return self._is_valid

    def metadata(self):
        return self._metadata

    def value(self):
        return self._ros_message

    def type(self):
        return self.extract_ros_type(self._ros_message)
