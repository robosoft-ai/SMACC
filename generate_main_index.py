#!/usr/bin/python
import os
 
packages_by_version = {}
package_list = []

# Set the directory you want to start from
rootDir = '..'
for dirName, subdirList, fileList in os.walk(rootDir):
    if "index.html" in fileList and dirName.endswith("c++"):
       #print('Found directory: %s' % dirName)
       fields = dirName.split("/")
       print(fields)
       ros_version = fields[1]
       package = fields[2]

       if not package in package_list:
           package_list.append(package)

       if ros_version in packages_by_version.keys():
          packages_by_version[ros_version].append(package)
       else:
          packages_by_version[ros_version] = [package]

    #for fname in fileList:
    #    print('\t%s' % fname)

print(packages_by_version)
ros_versions = packages_by_version.keys()

ros_versions = [v for v in ros_versions if v!="master"]

html = "<html><head></head><body>"
html +="""
<style>
#documentation {
  font-family: "Trebuchet MS", Arial, Helvetica, sans-serif;
  border-collapse: collapse;
  width: 100%;
}

#documentation td, #documentation th {
  border: 1px solid #ddd;
  padding: 8px;
}

#documentation tr:nth-child(even){background-color: #f2f2f2;}

#documentation tr:hover {background-color: #ddd;}

#documentation th {
  padding-top: 12px;
  padding-bottom: 12px;
  text-align: left;
  background-color: #4CAF50;
  color: white;
}
</style>
</head>
<body>
"""

html+= "<h1>"+ os.getcwd().split("/")[-1]+" documentation index </h1>"
html +="<table id=\"documentation\">"
html +="<tr><th>Package name</th>"
for version in ros_versions:
   html += "<th>"+ version + "</th>"
html +="</tr>"
for package in package_list:
   html += "<tr><td>"+ package+"</td>"
   for version in ros_versions:
      link = ""
      if package in packages_by_version[version]:
	link = version
      html+="<td>"+ "<a href=\""+version+"/"+package+"/html/c++/annotated.html"+"\">"+version+"</a></td>"
   html+="</tr>"

html +="</table>"




print ("list of packages: "+ str(package_list))
html += "</body></html>"

outfile=open("index.html","w")
outfile.write(html)
outfile.close()

print(packages_by_version)
