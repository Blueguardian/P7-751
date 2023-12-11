import pickle

pickle_contents = open("/home/christian/Drone_project/P7-751/dist_pickle.p", "rb")
pickle_contents = pickle.load(pickle_contents)
print(pickle_contents)