# -*- coding: utf-8 -*-

# Fits a 3D circle to post-processed data from the u-blox NEO-M8T FeatherWing
# processed by RTKLIB (RTKCONV and RTKPLOT).
# The .pos file from RTKPLOT is converted to .csv by POS_to_CSV.py and contains
# only x,y,z ECEF coordinates for data points with a Q of 1

# This code is based extensively on work by Miki at Meshlogic
# https://meshlogic.github.io/posts/jupyter/curve-fitting/fitting-a-circle-to-cluster-of-3d-points/

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.axes3d import Axes3D
import sys
import os
import string
import csv

print('CSV Circle Fitting')

filename = ''

if filename == '':
    # Check if the CSV filename was passed in argv
    if len(sys.argv) > 1: filename = sys.argv[1]

firstfile = ''
for root, dirs, files in os.walk("."):
    if len(files) > 0:
        if root == ".":
            for afile in files:
                if afile[-4:] == '.csv':
                    if firstfile == '': firstfile = afile

if filename == '': filename = raw_input('Enter the .csv filename (default: ' + firstfile + '): ') # Get the filename
if filename == '': filename = firstfile

print('Processing %s'%filename)

# Load the data
P = np.genfromtxt(filename,delimiter=',')

#-------------------------------------------------------------------------------
# Generate points on circle
# P(t) = r*cos(t)*u + r*sin(t)*(n x u) + C
#-------------------------------------------------------------------------------
def generate_circle_by_vectors(t, C, r, n, u):
    n = n/np.linalg.norm(n)
    u = u/np.linalg.norm(u)
    P_circle = r*np.cos(t)[:,np.newaxis]*u + r*np.sin(t)[:,np.newaxis]*np.cross(n,u) + C
    return P_circle

def generate_circle_by_angles(t, C, r, theta, phi):
    # Orthonormal vectors n, u, <n,u>=0
    n = np.array([np.cos(phi)*np.sin(theta), np.sin(phi)*np.sin(theta), np.cos(theta)])
    u = np.array([-np.sin(phi), np.cos(phi), 0.])
    
    # P(t) = r*cos(t)*u + r*sin(t)*(n x u) + C
    P_circle = r*np.cos(t)[:,np.newaxis]*u + r*np.sin(t)[:,np.newaxis]*np.cross(n,u) + C
    return P_circle

#-------------------------------------------------------------------------------
# FIT CIRCLE 2D
# - Find center [xc, yc] and radius r of circle fitting to set of 2D points
# - Optionally specify weights for points
#
# - Implicit circle function:
#   (x-xc)^2 + (y-yc)^2 = r^2
#   (2*xc)*x + (2*yc)*y + (r^2-xc^2-yc^2) = x^2+y^2
#   c[0]*x + c[1]*y + c[2] = x^2+y^2
#
# - Solution by method of least squares:
#   A*c = b, c' = argmin(||A*c - b||^2)
#   A = [x y 1], b = [x^2+y^2]
#-------------------------------------------------------------------------------
def fit_circle_2d(x, y, w=[]):
    
    A = np.array([x, y, np.ones(len(x))]).T
    b = (x**2.) + (y**2.)
    
    # Modify A,b for weighted least squares
    if len(w) == len(x):
        W = np.diag(w)
        A = np.dot(W,A)
        b = dnp.ot(W,b)
    
    # Solve by method of least squares
    c = np.linalg.lstsq(A,b)[0]
    
    # Get circle parameters from solution c
    xc = c[0]/2.
    yc = c[1]/2.
    r = np.sqrt(c[2] + (xc**2.) + (yc**2.))
    return xc, yc, r

#-------------------------------------------------------------------------------
# RODRIGUES ROTATION
# - Rotate given points based on a starting and ending vector
# - Axis k and angle of rotation theta given by vectors n0,n1
#   P_rot = P*cos(theta) + (k x P)*sin(theta) + k*<k,P>*(1-cos(theta))
#-------------------------------------------------------------------------------
def rodrigues_rot(P, n0, n1):
    
    # If P is only 1d array (coords of single point), fix it to be matrix
    if P.ndim == 1:
        P = P[np.newaxis,:]
    
    # Get vector of rotation k and angle theta
    n0 = n0/np.linalg.norm(n0)
    n1 = n1/np.linalg.norm(n1)
    k = np.cross(n0,n1)
    k = k/np.linalg.norm(k)
    theta = np.arccos(np.dot(n0,n1))
    
    # Compute rotated points
    P_rot = np.zeros((len(P),3))
    for i in range(len(P)):
        P_rot[i] = P[i]*np.cos(theta) + np.cross(k,P[i])*np.sin(theta) + k*np.dot(k,P[i])*(1-np.cos(theta))

    return P_rot

#-------------------------------------------------------------------------------
# ANGLE BETWEEN
# - Get angle between vectors u,v with sign based on plane with unit normal n
#-------------------------------------------------------------------------------
def angle_between(u, v, n=None):
    if n is None:
        return np.arctan2(np.linalg.norm(np.cross(u,v)), np.dot(u,v))
    else:
        return np.arctan2(np.dot(n,np.cross(u,v)), np.dot(u,v))

#-------------------------------------------------------------------------------
# - Make axes of 3D plot to have equal scales
# - This is a workaround to Matplotlib's set_aspect('equal') and axis('equal')
#   which were not working for 3D
#-------------------------------------------------------------------------------
def set_axes_equal_3d(ax):
    limits = np.array([ax.get_xlim3d(), ax.get_ylim3d(), ax.get_zlim3d()])
    spans = abs(limits[:,0] - limits[:,1])
    centers = np.mean(limits, axis=1)
    radius = 0.5 * max(spans)
    ax.set_xlim3d([centers[0]-radius, centers[0]+radius])
    ax.set_ylim3d([centers[1]-radius, centers[1]+radius])
    ax.set_zlim3d([centers[2]-radius, centers[2]+radius])

#-------------------------------------------------------------------------------
# (1) Fitting plane by SVD for the mean-centered data
# Eq. of plane is <p,n> + d = 0, where p is a point on plane and n is normal vector
#-------------------------------------------------------------------------------
P_mean = P.mean(axis=0)
P_centered = P - P_mean
U,s,V = np.linalg.svd(P_centered)

# Normal vector of fitting plane is given by 3rd column in V
# Note linalg.svd returns V^T, so we need to select 3rd row from V^T
normal = V[2,:]
d = -np.dot(P_mean, normal)  # d = -<p,n>

#-------------------------------------------------------------------------------
# (2) Project points to coords X-Y in 2D plane
#-------------------------------------------------------------------------------
P_xy = rodrigues_rot(P_centered, normal, [0,0,1])

#-------------------------------------------------------------------------------
# (3) Fit circle in new 2D coords
#-------------------------------------------------------------------------------
xc, yc, r = fit_circle_2d(P_xy[:,0], P_xy[:,1])

#--- Generate circle points in 2D
t = np.linspace(0, 2.*np.pi, 3600)
xx = xc + r*np.cos(t)
yy = yc + r*np.sin(t)

#-------------------------------------------------------------------------------
# (4) Transform circle center back to 3D coords
#-------------------------------------------------------------------------------
C = rodrigues_rot(np.array([xc,yc,0]), [0,0,1], normal) + P_mean
C = C.flatten()

#--- Generate points for fitting circle
t = np.linspace(0, 2.*np.pi, 3600)
u = P[0] - C
P_fitcircle = generate_circle_by_vectors(t, C, r, normal, u)

#-------------------------------------------------------------------------------
# Plot 2D
#-------------------------------------------------------------------------------

means = [np.mean(P[:,i]) for i in range(3)]
min_xlim = means[0] - (r * 1.1)
max_xlim = means[0] + (r * 1.1)
min_ylim = means[1] - (r * 1.1)
max_ylim = means[1] + (r * 1.1)
min_zlim = means[2] - (r * 1.1)
max_zlim = means[2] + (r * 1.1)

fig = plt.figure(figsize=(16,11))
alpha_pts = 0.2
figshape = (2,3)
ax = [None]*4
ax[0] = plt.subplot2grid(figshape, loc=(0,0), colspan=2)
ax[1] = plt.subplot2grid(figshape, loc=(1,0))
ax[2] = plt.subplot2grid(figshape, loc=(1,1))
ax[3] = plt.subplot2grid(figshape, loc=(1,2))
i = 0
ax[i].set_title('Fitting circle in 2D coords projected onto fitting plane')
ax[i].set_xlabel('x'); ax[i].set_ylabel('y');
ax[i].set_aspect('equal', 'datalim'); ax[i].margins(.1, .1)
ax[i].grid()
ax[i].get_xaxis().get_major_formatter().set_useOffset(False)
ax[i].get_yaxis().get_major_formatter().set_useOffset(False)
for tick in ax[i].get_xticklabels(): tick.set_rotation(45); tick.set_fontsize(10)
for tick in ax[i].get_yticklabels(): tick.set_rotation(45); tick.set_fontsize(10)
i = 1
ax[i].scatter(P[:,0], P[:,1], alpha=alpha_pts, label='Points')
ax[i].set_title('View X-Y')
ax[i].set_xlabel('x'); ax[i].set_ylabel('y');
ax[i].set(xlim=[min_xlim,max_xlim],ylim=[min_ylim,max_ylim],aspect=1)
ax[i].grid()
ax[i].get_xaxis().get_major_formatter().set_useOffset(False)
ax[i].get_yaxis().get_major_formatter().set_useOffset(False)
for tick in ax[i].get_xticklabels(): tick.set_rotation(45); tick.set_fontsize(10)
for tick in ax[i].get_yticklabels(): tick.set_rotation(45); tick.set_fontsize(10)
i = 2
ax[i].scatter(P[:,0], P[:,2], alpha=alpha_pts, label='Points')
ax[i].set_title('View X-Z')
ax[i].set_xlabel('x'); ax[i].set_ylabel('z'); 
ax[i].set(xlim=[min_xlim,max_xlim],ylim=[min_zlim,max_zlim],aspect=1)
ax[i].grid()
ax[i].get_xaxis().get_major_formatter().set_useOffset(False)
ax[i].get_yaxis().get_major_formatter().set_useOffset(False)
for tick in ax[i].get_xticklabels(): tick.set_rotation(45); tick.set_fontsize(10)
for tick in ax[i].get_yticklabels(): tick.set_rotation(45); tick.set_fontsize(10)
i = 3
ax[i].scatter(P[:,1], P[:,2], alpha=alpha_pts, label='Points')
ax[i].set_title('View Y-Z')
ax[i].set_xlabel('y'); ax[i].set_ylabel('z'); 
ax[i].set(xlim=[min_ylim,max_ylim],ylim=[min_zlim,max_zlim],aspect=1)
ax[i].grid()
ax[i].get_xaxis().get_major_formatter().set_useOffset(False)
ax[i].get_yaxis().get_major_formatter().set_useOffset(False)
for tick in ax[i].get_xticklabels(): tick.set_rotation(45); tick.set_fontsize(10)
for tick in ax[i].get_yticklabels(): tick.set_rotation(45); tick.set_fontsize(10)

ax[0].scatter(P_xy[:,0], P_xy[:,1], alpha=alpha_pts, label='Projected points')

ax[0].plot(xx, yy, 'k--', lw=2, label='Fitting circle')
ax[0].plot(xc, yc, 'k+', ms=10)
ax[0].legend()

ax[1].plot(P_fitcircle[:,0], P_fitcircle[:,1], 'k--', lw=2, label='Fitting circle')
ax[2].plot(P_fitcircle[:,0], P_fitcircle[:,2], 'k--', lw=2, label='Fitting circle')
ax[3].plot(P_fitcircle[:,1], P_fitcircle[:,2], 'k--', lw=2, label='Fitting circle')
ax[3].legend()

plt.show()

print('Fitting plane: n = %s' % np.array_str(normal, precision=4))
print('Fitting circle: center = %s, r = %.4f' % (np.array_str(C, precision=4), r))

latitude = np.degrees(np.arctan2(-normal[2], (normal[0]**2. + normal[1]**2.)**0.5))
longitude = np.degrees(np.arctan2(-normal[1], -normal[0]))
print('Circle would be horizontal at:   Latitude: %.2f   Longitude: %.2f'%(latitude,longitude))

#-------------------------------------------------------------------------------
# Plot 3D
#-------------------------------------------------------------------------------

fig = plt.figure(figsize=(16,10))
ax = fig.add_subplot(1,1,1,projection='3d')
ax.plot(*P.T, ls='', marker='o', alpha=0.3, label='Points')

#--- Plot fitting plane
xx, yy = np.meshgrid(np.linspace(min_xlim,max_xlim,11.), np.linspace(min_ylim,max_ylim,11.))
zz = (-normal[0]*xx - normal[1]*yy - d) / normal[2]
ax.plot_surface(xx, yy, zz, rstride=2, cstride=2, color='y' ,alpha=0.2, shade=False)

#--- Plot fitting circle
ax.plot(*P_fitcircle.T, color='k', ls='--', lw=2, label='Fitting circle')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()
rad = 'Circle Radius %.3fm'%r
plt.title(rad)
ax.get_xaxis().get_major_formatter().set_useOffset(False)
ax.get_yaxis().get_major_formatter().set_useOffset(False)
ax.zaxis.major.formatter.set_useOffset(False)
for tick in ax.get_xticklabels(): tick.set_fontsize(10)
for tick in ax.get_yticklabels(): tick.set_fontsize(10)
for tick in ax.get_zticklabels(): tick.set_fontsize(10)

ax.set_xlim3d(min_xlim,max_xlim)
ax.set_ylim3d(min_ylim,max_ylim)
ax.set_zlim3d(min_zlim,max_zlim)

plt.show()

#---------------------------------------------------------------------------------
# Calculate and plot minimum distances between data points and the fitting circle
#---------------------------------------------------------------------------------
print('Calculating minimum distances... (This could take a while!)')
min_dists = []
for point in P:
    min_dist = 1000.
    for fits in P_fitcircle:
        dist = ((fits[0] - point[0])**2. + (fits[1] - point[1])**2. + (fits[2] - point[2])**2.)**0.5
        if dist < min_dist: min_dist = dist
    min_dists.append(min_dist)
plt.hist(min_dists,'auto')
plt.xlabel('Distance (m)')
plt.ylabel('Frequency')
plt.title('Minimum distances from data points to fitting circle')
plt.grid(True)
plt.show()

mean_dist = np.mean(min_dists)
std_dist = np.std(min_dists)
print('Minimum distances from data points to fitting circle (m):   Mean %.4f   Std Dev %.4f'%(mean_dist,std_dist))



