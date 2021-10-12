from diagnostic_msgs.msg import KeyValue


def keyval_to_dict(keyval_list):
    if keyval_list is None or keyval_list == []:
        return {}
    out = {}
    for item in keyval_list:
        out[item.key] = item.value
    return out


def dict_to_keyval(in_dict):
    if in_dict is None:
        return []
    out = []
    for item in in_dict.items():
        out.append(KeyValue(*item))
    return out
