import plotext as plt
import time

path = "/home/piktessa/Stampa_terminal/lab_meccatronica.png"
#plt.download(plt.test_image_url, path)
#plt.image_plot(path, fast=True)
frames = 500
x1 = 10
y1 = 10
#plt.polygon(x = 20, y = 30, radius = 2)
#plt.rectangle()

plt.title('Lab')
plt.plot_size(100, 200)

start = time.perf_counter()
for i in range(frames):
    x1 = x1 + 1
    y1 = y1 + 1
    #plt.clear_plot()
    plt.clear_terminal()
    plt.cld()
    
    plt.polygon(x = x1, y = y1, sides = 100, radius = 10) # to simulate a circle
    plt.polygon(x = 150, y = 150, sides = 100, radius = 10) # to simulate a circle
    plt.xlim(0,500)
    plt.ylim(0,1000)
    
    
    plt.show()
    stop = time.perf_counter()
    print("Tempo trascorso = ", (stop-start))
    plt.sleep(1)
    start = time.perf_counter()
    #plt.delete_file(path)