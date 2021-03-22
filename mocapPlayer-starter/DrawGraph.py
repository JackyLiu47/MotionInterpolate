import matplotlib.pyplot as plt
import numpy as np

infile0 = "C:/Users/Jacky/Desktop/520hw2pic/pic/input.csv"
infile1 = "C:/Users/Jacky/Desktop/520hw2pic/pic/le.csv"
infile2 = "C:/Users/Jacky/Desktop/520hw2pic/pic/be.csv"
infile3 = "C:/Users/Jacky/Desktop/520hw2pic/pic/lq.csv"
infile4 = "C:/Users/Jacky/Desktop/520hw2pic/pic/bq.csv"
infile5 = "C:/Users/Jacky/Desktop/520hw2pic/pic/le2.csv"
infile6 = "C:/Users/Jacky/Desktop/520hw2pic/pic/be2.csv"
infile7 = "C:/Users/Jacky/Desktop/520hw2pic/pic/lq2.csv"
infile8 = "C:/Users/Jacky/Desktop/520hw2pic/pic/bq2.csv"
infile9 = "C:/Users/Jacky/Desktop/520hw2pic/pic/input2.csv"


Xin,Yin = np.loadtxt(infile0, delimiter = ",", unpack = True)
Xle,Yle = np.loadtxt(infile1, delimiter = ",", unpack = True)
Xbe,Ybe = np.loadtxt(infile2, delimiter = ",", unpack = True)
Xlq,Ylq = np.loadtxt(infile3, delimiter = ",", unpack = True)
Xbq,Ybq = np.loadtxt(infile4, delimiter = ",", unpack = True)
Xle2,Yle2 = np.loadtxt(infile5, delimiter = ",", unpack = True)
Xbe2,Ybe2 = np.loadtxt(infile6, delimiter = ",", unpack = True)
Xlq2,Ylq2 = np.loadtxt(infile7, delimiter = ",", unpack = True)
Xbq2,Ybq2 = np.loadtxt(infile8, delimiter = ",", unpack = True)
Xin2,Yin2 = np.loadtxt(infile9, delimiter = ",", unpack = True)
Xp = []
Yp = []
# graph1,2
# for i in range(len(Xle)):
#     if(Yle[i]==Ybe[i]):
#         Xp.append(Xle[i])
#         Yp.append(Yle[i])
# graph3,4
for i in range(len(Xle2)):
    if(Ybe2[i]==Ybq2[i]):
        Xp.append(Xle2[i])
        Yp.append(Yle2[i])


plt.title(r'graph4',fontsize=20)

plt.plot(Xin2,Yin2,label = 'Input',c = 'b')

# graph1
# plt.plot(Xle,Yle,label = 'LinearEuler',c = 'r')
# plt.plot(Xbe,Ybe,label = 'BezierEuler', c = 'g')

# graph2
# plt.plot(Xlq,Ylq,label = 'SLERP_Q',c = 'r')
# plt.plot(Xbq,Ybq,label = 'BezierSLERPQ', c = 'g')

# graph3
# plt.plot(Xle2,Yle2,label = 'LinearEuler',c = 'r')
# plt.plot(Xlq2,Ylq2,label = 'SLERP_Q', c = 'g')

# graph4
plt.plot(Xbe2,Ybe2,label = 'BezierEuler',c = 'r')
plt.plot(Xbq2,Ybq2,label = 'BezierSLERPQ', c = 'g')

plt.plot(Xp,Yp,'om',label = 'key point')
plt.legend(loc='upper right')

plt.show()
