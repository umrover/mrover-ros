from pymap3d.enu import geodetic2enu

# ref = (42.364379, -83.853656, 0)
ref = (42.325588, -83.839475, 265)
point = (42.318831, -84.625762, 302)
# point = (42.381420, -83.563339, 0)
# point = (42.431082, -83.483238, 253)

# ref = (42.235811, -83.694744, 254)
point = (42.293299, -83.710386, 264)

# earth radius at wilson center = 6368.762 km

# actual distance: 23.92km
# actual distance 2: 31.53km

# actual distance: 6.52km
# measured: 6.515km

e, n, u = geodetic2enu(*point, *ref, deg=True)
dist = (e**2 + n**2 + u**2)**(1/2)
print(e, n, u)
print(dist)