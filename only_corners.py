y_bird, x_bird, dim = img_orig.shape

def create_four_points_list(img_orig):              #this exists in the previous file. Only add global corner and corner definition
    global corner #add!!!
    height, width, shape = img_orig.shape
    w = int(width/8)
    h = int(height/8)
    four_points_list = []
    corner = [(0,y_bird), (x_bird,y_bird), (0,0), (x_bird,0)]   #add!!!
    for multiplier in range(8):
        for multiplier2 in range(8):
            four_point = [(w*multiplier2,h*(multiplier+1)),(w*(multiplier2+1),h*(multiplier+1)),(w*multiplier2,h*multiplier),(w*(multiplier2+1),h*multiplier)]
            four_points_list.append(four_point)
            
    return four_points_list


def transform_corner(pts, M):
    corner_original = []
    new_M = np.linalg.inv(M)
    
    pts = np.array(pts, dtype=np.float32)
    pts = pts.reshape(-1, 1, 2)
    pts2 = cv2.perspectiveTransform(pts, new_M).tolist()
    corner_original.append(pts2)
    return corner_original

corner_original = transform_corner(corner, M)
corner_list = []
for a in corner_original:
    for b in a:
        b[0][0] += x1
        b[0][1] += y1
        x_o , y_o = scale_coordinates_640_to_orig(b[0][0], b[0][1])
        corner_list += [(x_o, y_o)]
        