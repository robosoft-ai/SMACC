
import threading
import copy
import rospy

__all__ = ['UserData','Remapper']

class UserData(object):
    """smacc user data structure."""
    def __init__(self):
        self._data = {}
        self._locks = {}
        self.__initialized = True

    def update(self, other_userdata):
        """Combine this userdata struct with another.
        This overwrites duplicate keys with values from C{other_userdata}.
        """
        # Merge data
        self._data.update(other_userdata._data)

    def extract(self, keys, remapping):
        ud = UserData()
        reverse_remapping = {remapping[k]: k for k in remapping}
        if len(reverse_remapping) != len(remapping):
            rospy.logerr("smacc userdata remapping is not one-to-one: " + str(remapping))
        for k in keys:
            rmk = k
            if k in reverse_remapping:
                rmk = reverse_remapping[k]
            ud[rmk] = copy.copy(self[k])
        return ud

    def merge(self, ud, keys, remapping):
        for k in keys:
            rmk = k
            if k in remapping:
                rmk = remapping[k]
            self[rmk] = copy.copy(ud[k])

    def __getitem__(self, key):
        return self.__getattr__(key)

    def __setitem__(self, key, item):
        self._data[key] = item

    def keys(self):
        return list(self._data.keys())

    def __contains__(self,key):
        return key in self._data

    def __getattr__(self, name):
        """Override getattr to be thread safe."""
        if name[0] == '_':
            return object.__getattr__(self, name)
        if not name in self._locks.keys():
            self._locks[name] = threading.Lock()

        try:
            with self._locks[name]:
                temp = self._data[name]
        except:
            rospy.logerr("Userdata key '%s' not available. Available keys are: %s" % (name, str(list(self._data.keys()))))
            raise KeyError()

        return temp

    def __setattr__(self, name, value):
        """Override setattr to be thread safe."""
        # If we're still in __init__ don't do anything special
        if name[0] == '_' or '_UserData__initialized' not in self.__dict__:
            return object.__setattr__(self, name, value)

        if not name in self._locks.keys():
            self._locks[name] = threading.Lock()

        self._locks[name].acquire()
        self._data[name] = value
        self._locks[name].release()

# Const wrapper
def get_const(obj):
    """Get a const reference to an object if it has "user-defined" attributes."""
    if hasattr(obj,'__dict__'):
        rospy.logdebug("Making const '%s'" % str(obj))
        return Const(obj)
    else:
        return obj

class Const(object):
    """Wrapper that treats "user-defined" fields as immutable.
    
    This wrapper class is used when user data keys are specified as input keys,
    but not as output keys.
    """
    def __init__(self, obj):
        rospy.logdebug("Making const '%s'" % str(obj))
        self._obj = obj
        self.__initialized = True

    def __getattr__(self, name):
        rospy.logdebug("Getting '%s' from const wrapper." % name)
        attr = getattr(self._obj,name)
        return get_const(attr)

    def __getitem__(self, name):
        rospy.logdebug("Getting '%s' from const wrapper." % name)
        attr = self._obj[name]
        return get_const(attr)

    def __setattr__(self, name, value):
        if '_const__initialized' not in self.__dict__: 
            return object.__setattr__(self, name, value)
        rospy.logerr("Attempting to set '%s' but this member is read-only." % name)
        raise TypeError()

    def __delattr__(self, name):
        rospy.logerr("Attempting to delete '%s' but this member is read-only." % name)
        raise TypeError()

class Remapper(object):
    """Key-remapping proxy to a smacc userdata structure."""
    def __init__(self, ud, input_keys=[], output_keys=[], remapping={}):
        self._ud = ud
        self._input = input_keys
        self._output = output_keys
        self._map =  remapping
        self.__initialized = True

    def _remap(self, key):
        """Return either the key or it's remapped value."""
        if key in self._map:
            return self._map[key]
        return key

    def update(self, other_userdata):
        self._ud.update(other_userdata)

    def __getitem__(self, key):
        if key not in self._input:
            raise rospy.InvalidUserCodeError("Reading from smacc userdata key '%s' but the only keys that were declared as input to this state were: %s. This key needs to be declaread as input to this state. " % (key, self._input))
        if key not in self._output:
            return get_const(self._ud.__getitem__(self._remap(key)))
        return self._ud.__getitem__(self._remap(key))

    def __setitem__(self, key, item):
        if key not in self._output:
            rospy.logerr("Writing to smacc userdata key '%s' but the only keys that were declared as output from this state were: %s." % (key, self._output))
            return
        self._ud.__setitem__(self._remap(key),item)

    def keys(self):
        return [self._remap(key) for key in self._ud.keys() if key in self._input]

    def __contains__(self,key):
        if key in self._input:
            return self._remap(key) in self._ud
        else:
            return False

    def __getattr__(self, name):
        if name[0] == '_':
            return object.__getattr__(self, name)
        if name not in self._input:
            raise rospy.InvalidUserCodeError("Reading from smacc userdata key '%s' but the only keys that were declared as input to this state were: %s. This key needs to be declaread as input to this state. " % (name, self._input))
        if name not in self._output:
            return get_const(getattr(self._ud, self._remap(name)))
        return getattr(self._ud, self._remap(name))

    def __setattr__(self, name, value):
        if name[0] == '_' or '_Remapper__initialized' not in self.__dict__:
            return object.__setattr__(self, name, value)
        if name not in self._output:
            rospy.logerr("Writing to smacc userdata key '%s' but the only keys that were declared as output from this state were: %s." % (name, self._output))
            return None
        setattr(self._ud, self._remap(name), value)
