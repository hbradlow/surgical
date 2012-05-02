import sys
import os

filename = ""
if (len(sys.argv) > 1):
	filename = sys.argv[1]
def run(command):
	result_code = os.system(command)
	if result_code==0:
		print "Command (",command,") failed... Shutting down"
		exit()

command = str("../src/perception/kinfu_reconstruction " + filename)
result_code = run("mkdir tmp")
os.chdir("tmp")
run("kinfu_app")

run(command)
run("meshlabserver -i out_raw.ply -o out_recon.ply -s ../../src/perception/poisson.mlx")
run("meshlab out_recon.ply")

run("/home/henrybrad/Desktop/Ply2/ply/ply2ascii <out_recon.ply>out_tetgen.ply")
run("/home/henrybrad/Desktop/tetgen1.4.3/tetgen2 -O out_tetgen.ply")

run("meshlab out_tetgen.1.off")

run("../src/tests/test_cloth_grasping")
