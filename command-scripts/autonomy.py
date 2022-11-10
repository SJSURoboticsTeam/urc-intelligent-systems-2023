import math
def get_distance(lon1 ,lat1, lon2, lat2):

    R_KM = 6373.0
    R_MI = 3958.8
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)

    dis_lon = lon2 - lon1
    dis_lat = lat2 - lat1

    form1 = math.sin(dis_lat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dis_lon / 2)**2
    form2 = 2 * math.atan2(math.sqrt(form1), math.sqrt(1 - form1))

    distanceKM = R_KM * form2
    distanceMi = R_MI * form2
    return [distanceKM, distanceMi]