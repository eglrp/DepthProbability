import sys, os


exe_path = "/usr/local/bin/OpenMVS/InterfaceVisualSFM"


def main(argv):
    if len(argv) == 1:
        print "input folder please"
        return

    print "working folder is: "+argv[1]
    command = exe_path+" -i "+argv[1]+"/result.nvm -o "+argv[1]+"/sparse.mvs"+" -w "+argv[1]
    os.system(command)

if __name__ == '__main__':
    main(sys.argv)