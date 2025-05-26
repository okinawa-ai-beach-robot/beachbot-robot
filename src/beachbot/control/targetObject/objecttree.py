class ObjectTree:
    def __init__(self,name):
        self.name = name
        self.data = None
        self.children = []
        self.parent = None
    def addChild(self,child):
        self.children.append(child)
    def removeChild(self,child):
        self.children.remove(child)
    def removeParent(self,parent):
        self.parent = None
    def addParent(self,parent):
        self.parent = parent
    def setData(self,data):
        self.data = data
    def getFirstChild(self):
        try:
            print(self.children[0])
        except:
            print ("the tree branch is empty")
        return self.children[0];
    def getParent(self):
        if self.parent == None:
            print("the parent tree is empty")
        else:
            return self.parent
    def swithChildOrder(self, i, j):
        if i >len(self.children) or j > len(self.children):
            print ("index out of range")
        else:
            temperary = self.children[i]
            self.children[i] = self.children[j]
            self.children[j] = temperary
    def getchild(self,i):
        if i < len(self.children):
            return self.children[i]
        else:
            print("index out of range")


