# def im_to_cam(u, v, d, fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7, factor = 1):
def im_to_cam(u, v, d, fx = 616.9660034179688, fy = 616.8399047851562, \
	cx = 328.9248962402344, cy = 230.74755859375, factor = 20):
    Z = d / factor
    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy

    return [X, Y, Z]
