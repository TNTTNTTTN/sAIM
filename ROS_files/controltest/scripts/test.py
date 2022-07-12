from darknetinit import Darknetinit

class testing(Darknetinit):
    def __init__(self):
        super(testing,self).__init__()
        print(self.point_cloud)


a =testing()