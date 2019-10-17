import re

class TypeInfo:
    def __init__(self, tkey, codedtype, finaltype):
        self.tkey = tkey
        self.codedtype = codedtype
        self.finaltype = finaltype
        self.template_parameters = []

    def __str__(self):
        return self.tkey + ":" + self.finaltype

def getRootTypeInfo(inputtext):
    ok = False
    typecount = 0
    typesdict = {}
    originalinputtext = inputtext

    # this loop flatterns the template type tree
    while not ok:
        simpletypeRE = r"[^<>,\s]+<[^<>]+>"
        print ("input: " + inputtext)

        matches = [m for m in enumerate(re.finditer(simpletypeRE, inputtext))]
        if len(matches) == 0:
            if len(typesdict) == 0:
                typekey = "$T" + str(typecount)
                typesdict[typekey] = inputtext
            break

        for i, m in matches:
            tstr = m.group(0)

            typekey = "$T" + str(typecount)
            inputtext = inputtext.replace(tstr, typekey)
            print("updating input text: " + inputtext)
            typecount += 1
            typesdict[typekey] = tstr

    # once the tree is flatterned we may get some template with flat types
    simpletypeRE = r"<(.*)>"

    allbasetypes = set()
    for tkey in typesdict.keys():
        flat = typesdict[tkey]
        print flat
        startindex = flat.index("<")
        flat = flat[startindex + 1:-1]
        basetypes = [t.strip() for t in flat.split(",")]
        for b in basetypes:
            if not "$" in b:
                allbasetypes.add(b)

    for b in allbasetypes:
        typesdict[b] = b

    def replace_back(roottype, typesdict):
        # replace back
        while "$" in roottype:
            #print roottype
            for tkey in typesdict:
                tval = typesdict[tkey]
                roottype = roottype.replace(tkey, tval)

        return roottype

    types = []
    for tkey in typesdict:
        finaltype = replace_back(typesdict[tkey], typesdict)
        t = TypeInfo(tkey, typesdict[tkey], finaltype)
        types.append(t)

        print t

    print (typesdict)
    roottype = [t for t in types if t.finaltype == originalinputtext][0]

    print "---------------------------------"

    # fill template parameters
    for t in types:
        for t2 in types:
            if t2.tkey in t.codedtype:
                index = t.codedtype.index(t2.tkey)
                t.template_parameters.append((index, t2))

        t.template_parameters = [x[1] for x in sorted(
            t.template_parameters, key=lambda e: e[0])]

    return roottype
    
inputtext = "smacc::transition<Ev1<LidarSensor<sensor_msgs::LaserScan, std::allocator<none>>, EVTAG>, State1, TAG>"
roottypeinfo = getRootTypeInfo(inputtext)

print types
