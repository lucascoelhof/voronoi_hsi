import traceback


def error_str(func_name, exception):
    return func_name + " failed with exception: " + str(exception) + "\n" + str(traceback.print_exc())


def print_dict(dictn):
    for elem in dictn:
        print(elem, repr(dictn[elem]))


def dump_attrs(obj):
    for attr in dir(obj):
        print("obj.%s" % attr)