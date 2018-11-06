#coding:utf-8
import os

if os.path.exists("./bin/list.txt"):
	infile = open("./bin/list.txt")
else:
	print("Can not find inputfile")
outfile = open("files_list.txt",'w')

all_lines = infile.readlines()
print("Starting...")

for line in all_lines:
	outfile.write(line[:-5]+"\n")
	#print(line[:-5])

print("Completed")
infile.close()
outfile.close()
