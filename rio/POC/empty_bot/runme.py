from time import sleep


def myFunction():
    print(2 + 2)
    print("Hello this is a print")

def theLoop():
    myFunction()
    input()
    theLoop()

def main():
    import jurigged;
    jurigged.watch("/home/lvuser/py/*.py")
    print("Main ran???")
    theLoop()

if __name__ == "__main__":
    main()