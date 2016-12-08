#!usr/bin/env python
#Initialize

block_num = 3
x1 = 0.4
y1 = 0.9
x2 = 0.4
y2 = 0.7
x3 = 0.5
y3 = 0.6
block_array = [[x1, y1], [x2, y2], [x3, y3]]

#distance tolerance
tol = 0.05

#0.2 meters
ws_side_length = 0.2

#flag, 1 if candidate location is too close to existing location, otherwise 0
too_close = 1

#three shuffles
for i in range(1,3):
    #shuffle 3 blocks
    for j in range (1, block_num):
        while too_close == 1
            x_rand_seed = rand
            y_rand_seed = rand
            x_new = x_rand_seed*ws_side_length
            y_new = y_rand_seed*ws_side_length
            for k in range (1, block_num):
               dist(k) = sqrt((x_new - block_array[k,1])**2 + (y_new - block_array[k,2])**2)

            if dist[1] > tol && dist[2] > tol && dist[3] > tol:
                #change the flag, trigger exit from while loop
                too_close = 0
            else
                too_close = 1

            print dist
        block_array[j,1] = x_new
        block_array[j,2] = y_new

        too_close = 1

    print block_array
