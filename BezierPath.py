import math


class BezierPath:
    # Assumption: WPList is a list of [ [x coord, y coord]  ]

    def __init__(self, CPs, obst):
        self.CPs = CPs
        self.obst = obst
        self.magn = 0.5
        self.handles = []

    def computeHandles(self):
        for i in range(len(self.CPs)):
            if i == 0:
                hx = self.CPs[i][0]
                hy = self.CPs[i][1]
                self.handles.append([hx, hy])

            elif i != len(self.CPs)-1:
                h1x = self.CPs[i][0] - \
                    (self.CPs[i+1][0] - self.CPs[i][0])*self.magn
                h1y = self.CPs[i][1] - \
                    (self.CPs[i+1][1] - self.CPs[i][1])*self.magn

                h2x = self.CPs[i][0] + (self.CPs[i][0] - h1x)

                h2y = self.CPs[i][1] + (self.CPs[i][1] - h1y)
                self.handles.append([h1x, h1y])
                self.handles.append([h2x, h2y])
            else:
                self.handles.append(
                    [self.CPs[len(self.CPs)-1][0], self.CPs[len(self.CPs)-1][1]])

    def computeWPs(self):
        wps = []
        for i in range(len(self.CPs)):
            if(i < len(self.CPs)-1):
                bpoints = self.courbe_bezier_3(
                    [self.CPs[i], self.handles[i*2], self.handles[i*2+1], self.CPs[i+1]], 10)
                for j in range(len(bpoints)):
                    wps.append(bpoints[j])
        return self.ObstacleCorrection(wps)

    def getDist(self, p0, p1):
        return math.sqrt(math.pow((p0[0]-p1[0]), 2)+math.pow((p0[1]-p1[1]), 2))

    def ObstacleCorrection(self, wps):
        for i in range(len(wps)):
            if(self.getDist(wps[i], self.obst) < 1):
                dirVecx = wps[i][0] - self.obst[0]
                dirVecy = wps[i][1] - self.obst[1]
                dirVecx /= self.getDist([0, 0], [dirVecx, dirVecy])
                dirVecy /= self.getDist([0, 0], [dirVecx, dirVecy])
                wps[i][0] += dirVecx
                wps[i][1] += dirVecy

        return wps

    def combinaison_lineaire(self, A, B, u, v):
        return [A[0]*u+B[0]*v, A[1]*u+B[1]*v]

    def interpolation_lineaire(self, A, B, t):
        return self.combinaison_lineaire(A, B, t, 1-t)

    def point_bezier_3t(self, points_control, t):
        x = (1-t)**2
        y = t*t
        A = self.combinaison_lineaire(
            points_control[0], points_control[1], (1-t)*x, 3*t*x)
        B = self.combinaison_lineaire(
            points_control[2], points_control[3], 3*y*(1-t), y*t)
        return [A[0]+B[0], A[1]+B[1]]

    def courbe_bezier_3(self, points_control, N):
        dt = 1.0/N
        t = dt
        points_courbe = [points_control[0]]
        while t < 1.0:
            points_courbe.append(self.point_bezier_3t(points_control, t))
            t += dt
            # points_courbe.append(points_control[3])
        return points_courbe
