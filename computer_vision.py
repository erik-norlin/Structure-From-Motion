import cv2
import matplotlib as mpl
import matplotlib.image as mpimg
from matplotlib.pyplot import cm
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from numpy import linalg as LA
import numpy as np


def load_image(path, multi=False):
    if multi:
        img = []    
        for im in path:
            img.append(cv2.imread(im)) 
        img = np.array(img)
    else:
        img = cv2.imread(path)  
    return img 


def dehomogenize(x):
    x_deh = x/x[-1]
    return x_deh


def homogenize(x, multi=False):
    if multi:
        ones = np.ones(np.size(x,1))
        x_h = np.vstack([x, ones])
    else:
        x_h = np.append(x, 1)
    return x_h


def get_canonical_camera():
    P = np.concatenate((np.eye(3), np.zeros(3)[:,np.newaxis]), 1)
    return P


def compute_sift_points(img1, img2, marg, flann=False, verbose=False):
    img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
    img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

    sift = cv2.SIFT_create()
    kp1, des1 = sift.detectAndCompute(img1, None)
    kp2, des2 = sift.detectAndCompute(img2, None)

    if flann:
        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks=50) 

        flann = cv2.FlannBasedMatcher(index_params, search_params)
        matches = flann.knnMatch(des1, des2, k=2)
    else:
        bf = cv2.BFMatcher()
        matches = bf.knnMatch(des1, des2, k=2)

    good_matches = []
    for m, n in matches:
        if m.distance < marg*n.distance:
            good_matches.append([m])

    x1 = np.stack([kp1[match[0].queryIdx].pt for match in good_matches],1)
    x2 = np.stack([kp2[match[0].trainIdx].pt for match in good_matches],1)
    x1 = homogenize(x1, multi=True)
    x2 = homogenize(x2, multi=True)

    des1 = np.stack([des1[match[0].queryIdx] for match in good_matches],0)
    des2 = np.stack([des2[match[0].trainIdx] for match in good_matches],0)

    if verbose:
        print('Number of matches:', np.size(matches,0))
        print('Number of good matches:', np.size(x1,1))

    return x1, x2, des1, des2


def compute_sift_points_TR(x1, des1, img2, marg, flann=False, verbose=False):
    img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

    sift = cv2.SIFT_create()
    kp2, des2 = sift.detectAndCompute(img2, None)

    if flann:
        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks=50) 

        flann = cv2.FlannBasedMatcher(index_params, search_params)
        matches = flann.knnMatch(des1, des2, k=2)
    else:
        bf = cv2.BFMatcher()
        matches = bf.knnMatch(des1, des2, k=2)

    good_matches = []
    for m, n in matches:
        if m.distance < marg*n.distance:
            good_matches.append([m])

    x_idx = np.array([match[0].queryIdx for match in good_matches])
    x1 = np.stack([x1[:,match[0].queryIdx] for match in good_matches],1)
    x2 = np.stack([kp2[match[0].trainIdx].pt for match in good_matches],1)
    x2 = homogenize(x2, multi=True)

    if verbose:
        print('Number of matches:', np.size(matches,0))
        print('Number of good matches:', np.size(x1,1))

    return x1, x2, x_idx


def enforce_essential(E):
    U, _, VT = LA.svd(E, full_matrices=False)
    if LA.det(U @ VT) < 0:
        VT = -VT
    E =  U @ np.diag([1,1,0]) @ VT
    return E


def estimate_E_DLT(img1_pts_norm, img2_pts_norm, enforce=False):
    n = np.size(img1_pts_norm,1)
    M = []

    for i in range(n):

        x = img1_pts_norm[:,i]
        y = img2_pts_norm[:,i]
        m = np.outer(y, x).flatten()
        M.append([m])

    M = np.concatenate(M, 0)
    _, _, VT = LA.svd(M, full_matrices=False)
    E = VT[-1,:].reshape(3,3)
    if enforce:
        E = enforce_essential(E)

    E = E/E[-1,-1]
    return E


def compute_E_validity(E):
    rank = LA.matrix_rank(E)
    valid = True if rank == 2 else False
    return valid


def compute_epipolar_lines(F, x1, x2):
    l2 = F @ x1
    l1 = F.T @ x2
    return l1, l2


def compute_point_line_distance_2D(l, p, multi=False):
    if multi:
        a = l[0,:]
        b = l[1,:]
        c = l[2,:]

        x = p[0,:]
        y = p[1,:]
    else:
        a = l[0]
        b = l[1]
        c = l[2]

        x = p[0]
        y = p[1]
    
    distance_arr = np.abs(a*x + b*y + c) / (a**2 + b**2)**0.5
    return distance_arr


def compute_epipolar_errors(F, x1, x2):
    l1, l2 = compute_epipolar_lines(F, x1, x2)
    distance1_arr = compute_point_line_distance_2D(l1, x1, multi=True)
    distance2_arr = compute_point_line_distance_2D(l2, x2, multi=True)
    return distance1_arr, distance2_arr


def compute_E_inliers(E, x1_norm, x2_norm, err_threshold):

    distance1_arr, distance2_arr = compute_epipolar_errors(E, x1_norm, x2_norm)
    inliers = ((distance1_arr**2 + distance2_arr**2) / 2) < err_threshold**2
    n_inliers = np.sum(inliers)
    epsilon_E = n_inliers / x1_norm.shape[1]
    return epsilon_E, inliers


def compute_ransac_iterations(alpha, epsilon, s, min_its, max_its, scale):
    T = scale * np.ceil(np.log(1-alpha) / np.log(1-epsilon**s))
    if np.isinf(T) or T > max_its:
        T = max_its
    elif T < min_its:
        T = min_its
    return T


def estimate_H_DLT(img1_pts, img2_pts):
    n = np.size(img1_pts,1)
    M = []

    for i in range(n):

        x = img1_pts[0,i]
        y = img1_pts[1,i]

        u = img2_pts[0,i]
        v = img2_pts[1,i]

        m = np.array([[x, y, 1, 0, 0, 0, -u*x, -u*y, -u],
                      [0, 0, 0, x, y, 1, -v*x, -v*y, -v]])

        M.append(m)

    M = np.concatenate(M, 0)
    _, _, VT = LA.svd(M, full_matrices=False)
    H = np.stack([VT[-1, i:i+3] for i in range(0, 9, 3)], 0)

    return H


def compute_point_point_distance(x_proj, x_img):
    distance_arr = LA.norm(x_proj - x_img, axis=0)
    return distance_arr


def unitize(a, b):
    denom = 1.0 / np.sqrt(a**2 + b**2)
    ra = a * denom
    rb = b * denom
    return ra, rb


def homography_to_RT(H, x1, x2):
    # Check the right sign for H
    if LA.det(H) < 0:
        H *= -1 
        
    N = x1.shape[1]
    if x1.shape[0] != 3:
        x1 = np.vstack([x1, np.ones((1, N))])
    if x2.shape[0] != 3:
        x2 = np.vstack([x2, np.ones((1, N))])

    positives = np.sum(np.sum(x2 * (H @ x1), axis=0) > 0)
    if positives < (N / 2):
        H *= -1

    U, S, VT = np.linalg.svd(H, full_matrices=False)
    V = VT.T
    s1 = S[0] / S[1]
    s3 = S[2] / S[1]
    zeta = s1 - s3
    a1 = np.sqrt(1 - s3**2)
    b1 = np.sqrt(s1**2 - 1)
    a, b = unitize(a1, b1)
    c, d = unitize(1+s1*s3, a1*b1)
    e, f = unitize(-b/s1, -a/s3)
    v1, v3 = V[:, 0], V[:, 2]
    n1 = b * v1 - a * v3
    n2 = b * v1 + a * v3
    R1 = U @ np.array([[c, 0, d], [0, 1, 0], [-d, 0, c]]) @ VT
    R2 = U @ np.array([[c, 0, -d], [0, 1, 0], [d, 0, c]]) @ VT
    t1 = e * v1 + f * v3
    t2 = e * v1 - f * v3
    if n1[2] < 0:
        t1 = -t1
        n1 = -n1
    if n2[2] < 0:
        t2 = -t2
        n2 = -n2

    # Move from Triggs' convention H = R*(I - t*n') to H&Z notation H = R - t*n'
    t1 = R1 @ t1
    t2 = R2 @ t2

    # Verify that we obtain the initial homography back
    # H /= S[1]
    # print(np.linalg.norm(R1 - zeta * np.outer(t1, n1) - H), np.linalg.norm(R2 - zeta * np.outer(t2, n2) - H))

    return R1, t1, R2, t2

    # Example usage:
    # H is the homography matrix, x1 and x2 are the corresponding 2D points
    # R1, t1, R2, t2 = homography_to_RT(H, x1, x2)


def create_skew_symmetric_matrix(t):
    T = np.array([[0, -t[2], t[1]],
                  [t[2], 0, -t[0]],
                  [-t[1], t[0], 0]])
    return T


def compute_E_from_R_and_T(R, T):
    E = create_skew_symmetric_matrix(T) @ R
    return E


def verbose_E_robust(t, T_E, T_H, epsilon_E, epsilon_H, inliers, method):
    print('Iteration:', t, 'T_E:', T_E, 'T_H:', T_H, 'epsilon_E:', np.round(epsilon_E, 2), 'epsilon_H:', np.round(epsilon_H, 2), 'No. inliers:', np.sum(inliers), 'From:', method)


def estimate_E_robust(K, x1_norm, x2_norm, min_its, max_its, scale_its, alpha, err_threshold_px, essential_matrix=True, homography=True, verbose=False):
    
    err_threshold = err_threshold_px / K[0,0]
    best_E = None
    best_inliers = None
    n_points = x1_norm.shape[1]
    n_E_samples = 8
    n_H_samples = 4
    best_epsilon_E = 0
    best_epsilon_H = 0
    T_E = max_its
    T_H = max_its

    t = 0
    while t < T_E and t < T_H:
        t += 1

        if essential_matrix:
            rand_mask = np.random.choice(n_points, n_E_samples, replace=False)
            E = estimate_E_DLT(x1_norm[:,rand_mask], x2_norm[:,rand_mask], enforce=True)
            E_valid = compute_E_validity(E)

            if E_valid:
                epsilon_E, inliers = compute_E_inliers(E, x1_norm, x2_norm, err_threshold)
                    
                if epsilon_E > best_epsilon_E:
                    best_E = np.copy(E)
                    best_inliers = np.copy(inliers)
                    best_epsilon_E = epsilon_E
                    T_E = compute_ransac_iterations(alpha, best_epsilon_E, n_E_samples, min_its, max_its, scale_its)

                    if verbose:
                        verbose_E_robust(t, T_E, T_H, best_epsilon_E, best_epsilon_H, best_inliers, method='E 8-point alg.')
        
        if homography:
            rand_mask = np.random.choice(n_points, n_H_samples, replace=False)
            H = estimate_H_DLT(x1_norm[:,rand_mask], x2_norm[:,rand_mask])
            x2_norm_proj = dehomogenize(H @ x1_norm)
            distance_arr = compute_point_point_distance(x2_norm_proj, x2_norm)
            inliers = distance_arr < (3*err_threshold)
            n_inliers = np.sum(inliers)
            epsilon_H = n_inliers / n_points

            if epsilon_H > best_epsilon_H:
                
                R1, T1, R2, T2 = homography_to_RT(H, x1_norm, x2_norm)
                E1 = compute_E_from_R_and_T(R1, T1)
                E2 = compute_E_from_R_and_T(R2, T2)

                E1_valid = compute_E_validity(E1)
                E2_valid = compute_E_validity(E2)

                if E1_valid:
                    epsilon_E, inliers = compute_E_inliers(E1, x1_norm, x2_norm, err_threshold)
                        
                    if epsilon_E > best_epsilon_E:
                        best_E = np.copy(E1)
                        best_inliers = np.copy(inliers)
                        best_epsilon_E = epsilon_E
                        best_epsilon_H = epsilon_H
                        T_E = compute_ransac_iterations(alpha, best_epsilon_E, n_E_samples, min_its, max_its, scale_its)
                        T_H = compute_ransac_iterations(alpha, best_epsilon_H, n_H_samples, min_its, max_its, scale_its)

                        if verbose:
                            verbose_E_robust(t, T_E, T_H, best_epsilon_E, best_epsilon_H, best_inliers, method='H 4-point alg.')

                if E2_valid:
                    epsilon_E, inliers = compute_E_inliers(E2, x1_norm, x2_norm, err_threshold)
                        
                    if epsilon_E > best_epsilon_E:
                        best_E = np.copy(E2)
                        best_inliers = np.copy(inliers)
                        best_epsilon_E = epsilon_E
                        best_epsilon_H = epsilon_H
                        T_E = compute_ransac_iterations(alpha, best_epsilon_E, n_E_samples, min_its, max_its, scale_its)
                        T_H = compute_ransac_iterations(alpha, best_epsilon_H, n_H_samples, min_its, max_its, scale_its)
                        
                        if verbose:
                            verbose_E_robust(t, T_E, T_H, best_epsilon_E, best_epsilon_H, best_inliers, method='H 4-point alg.')
        
    print('Bailout at iteration:', t)
    return best_E, best_inliers


def triangulate_3D_point_DLT(P1, P2, img1_pts, img2_pts):
    n = np.size(img1_pts,1)
    X = []

    for i in range(n):

        x1 = img1_pts[0,i]
        y1 = img1_pts[1,i]
        
        x2 = img2_pts[0,i]
        y2 = img2_pts[1,i]

        M = np.array([[P1[0,0]-x1*P1[2,0], P1[0,1]-x1*P1[2,1], P1[0,2]-x1*P1[2,2], P1[0,3]-x1*P1[2,3]],
                      [P1[1,0]-y1*P1[2,0], P1[1,1]-y1*P1[2,1], P1[1,2]-y1*P1[2,2], P1[1,3]-y1*P1[2,3]],
                      [P2[0,0]-x2*P2[2,0], P2[0,1]-x2*P2[2,1], P2[0,2]-x2*P2[2,2], P2[0,3]-x2*P2[2,3]],
                      [P2[1,0]-y2*P2[2,0], P2[1,1]-y2*P2[2,1], P2[1,2]-y2*P2[2,2], P2[1,3]-y2*P2[2,3]]])

        _, _, VT = LA.svd(M, full_matrices=False)
        X.append(VT[-1,:])
        
    X = dehomogenize(np.stack(X,1))
    return X # in P3


def compute_feasible_points(P1, P2, X, percentile, ransac=True):
    if ransac:
        x1 = P1 @ X
        x2 = P2 @ X
        x1_filter = x1[-1,:] > 0
        x2_filter = x2[-1,:] > 0

    X_bar = np.mean(X, axis=1)
    X_norm = LA.norm(X - X_bar[:,None], axis=0)
    norm_percentile = np.percentile(X_norm, percentile)
    outlier_filter = X_norm < norm_percentile

    if ransac:
        feasible_pts = x1_filter * x2_filter * outlier_filter
    else:
        feasible_pts = outlier_filter
    return feasible_pts


def extract_skew_symmetric_vector(T):
    t = np.array([T[2,1], T[0,2], T[1,0]])
    return t


def extract_P_from_E(E):

    U, _, VT = LA.svd(E, full_matrices=False)

    if LA.det(U @ VT) < 0:
        VT = -VT

    W = np.array([[0,1,0],[-1,0,0],[0,0,1]])
    Z = np.array([[0,-1,0],[1,0,0],[0,0,0]])

    S1 = U @ Z @ U.T
    S2 = U @ Z.T @ U.T

    R1 = U @ W @ VT
    R2 = U @ W.T @ VT

    t1 = extract_skew_symmetric_vector(S1)
    t2 = extract_skew_symmetric_vector(S2)

    P1 = np.concatenate((R1, t1[:, np.newaxis]), 1)
    P2 = np.concatenate((R1, t2[:, np.newaxis]), 1)
    P3 = np.concatenate((R2, t1[:, np.newaxis]), 1)
    P4 = np.concatenate((R2, t2[:, np.newaxis]), 1)

    P_arr = np.array([P1, P2, P3, P4])
    return P_arr


def compute_triangulated_X_from_extracted_P2_solutions(P1, P2_arr, x1_norm, x2_norm):
    X_arr = np.array([triangulate_3D_point_DLT(P1, P2, x1_norm, x2_norm) for P2 in P2_arr])
    return X_arr


def transform(P, x):
    y = P @ x
    return y


def extract_valid_camera_and_points(P1, P_arr, X_arr, verbose=False):
        
    x1_arr = np.array([transform(P1, X) for X in X_arr])
    valid_coords_P1 = np.array([np.sum(x[-1] > 0) for x in x1_arr])

    x2_arr = np.array([transform(P_arr[i], X_arr[i]) for i in range(np.size(P_arr, 0))])
    valid_coords_P2 = np.array([np.sum(x[-1] > 0) for x in x2_arr]) 

    valid_coords = valid_coords_P1 + valid_coords_P2
    valid_coords_ind = np.argmax(valid_coords)
    X_valid = X_arr[valid_coords_ind]
    P2_valid = P_arr[valid_coords_ind]
    
    if verbose:
        print('No. valid coords for each camera pair:', valid_coords)
        print('Argmax(P2_arr):', valid_coords_ind)
    
    return P2_valid, X_valid


def normalize_vector(v):
    norm_v = v/(v @ v)**0.5
    return norm_v


def compute_normalized_principal_axis(P):
    M = P[:,:3]
    m3 = P[-1,:3]
    v = LA.det(M) * m3
    return normalize_vector(v)


def compute_camera_center(P):
    M = P[:,:3]
    P4 = P[:,-1]
    C = -1*(LA.inv(M) @ P4)
    return C


def compute_camera_center_and_normalized_principal_axis(P, multi=False):

    if multi:
        C_arr = np.array([homogenize(compute_camera_center(P[i])) for i in range(np.size(P,0))])
        axis_arr = np.array([homogenize(compute_normalized_principal_axis(P[i])) for i in range(np.size(P,0))])
    else:
        C_arr = np.array([homogenize(compute_camera_center(P))])
        axis_arr = np.array([homogenize(compute_normalized_principal_axis(P))])

    if np.size(C_arr, 0) != 4: # (n,4) => (4,n)
        C_arr = C_arr.T
        axis_arr = axis_arr.T
    
    return C_arr, axis_arr


def compute_absolute_rotations(rel_rots, origin_idx, verbose=False):
    
    abs_rots = [rel_rots[0]]
    for i in range(len(rel_rots)-1):

        Ri = abs_rots[i]
        R2 = rel_rots[i+1]
        if LA.det(R2) < 0:
            print('WARNING: det(R{}) < 0, not a rotation!'.format(i), LA.det(R2))
            R2 = -R2
        U, _, VT = LA.svd(R2, full_matrices=False)
        R2 = U @ VT
        Rj = R2 @ Ri
        abs_rots.append(Rj)

    R0 = abs_rots[origin_idx]    
    for i in range(len(abs_rots)):

        Ri = abs_rots[i]
        Ri = LA.inv(R0) @ Ri
        abs_rots[i] = Ri

        if verbose:
            print('det(R{}):'.format(i), LA.det(Ri))
        
    return np.array(abs_rots)


def estimate_T_least_squares(R, X_pts, x_pts):
    n = x_pts.shape[1]
    A = []
    B = []

    for i in range(n):

        x1 = x_pts[0,i]
        x2 = x_pts[1,i]

        X = R @ X_pts[:,i]
        X1 = X[0]
        X2 = X[1]
        X3 = X[2]

        a = np.array([[1, 0, -x1],
                      [0, 1, -x2]])
        
        b = np.array([[X3*x1 - X1],
                      [X3*x2 - X2]])

        A.append(a)
        B.append(b)

    A = np.concatenate(A, 0)
    B = np.concatenate(B, 0)
    T = LA.lstsq(A, B, rcond=None)[0].flatten()
    return T


def estimate_T_robust(K, R, X, x_norm, min_its, max_its, scale_its, alpha, err_threshold_px, verbose=False):
    
    err_threshold = err_threshold_px / K[0,0]
    best_T = np.full(3, np.nan)
    best_inliers = np.zeros(x_norm.shape[1], dtype=bool)
    best_epsilon = 0
    n_points = x_norm.shape[1]
    n_samples = 2
    ransac_its = max_its

    t = 0
    while t < ransac_its:
        t += 1

        rand_mask = np.random.choice(n_points, n_samples, replace=False)
        T = estimate_T_least_squares(R, X[:,rand_mask], x_norm[:,rand_mask])

        x_norm_proj = dehomogenize(R @ X + T[:,None])
        distance_arr = compute_point_point_distance(x_norm_proj, x_norm)
        inliers = distance_arr < err_threshold
        n_inliers = np.sum(inliers)
        epsilon = n_inliers / n_points

        if epsilon > best_epsilon:
            best_T = np.copy(T)
            best_inliers = np.copy(inliers)
            best_epsilon = epsilon
            ransac_its = compute_ransac_iterations(alpha, best_epsilon, n_samples, min_its, max_its, scale_its)
            if verbose:
                print('Iteration:', t, 'T:', ransac_its, 'epsilon:', np.round(best_epsilon, 2), 'No. inliers:', np.sum(inliers))
    
    print('Bailout at iteration:', t)
    return best_T, best_inliers


def plot_cameras_and_axes(ax, C_list, axis_list, s, valid_idx, col):

    for i in range(np.size(C_list,1)):
        C = C_list[:,i]
        axis = axis_list[:,i]
        ax.plot(C[0], C[1], C[2], 'o', color=col[i],  label='Camera {}'.format(valid_idx[i]+1), alpha=0.7)

        x_axis = C[0] + s*axis[0]
        y_axis = C[1] + s*axis[1]
        z_axis = C[2] + s*axis[2]

        ax.plot([x_axis, C[0]], [y_axis, C[1]], [z_axis, C[2]], '-', color=col[i], lw=3, alpha=0.7)


def plot_cameras_and_3D_points(X_arr, C_arr, axis_arr, s, title, valid_idx, multi=False):
    fig = plt.figure(figsize=(12,8))
    ax = plt.axes(projection='3d')
    col = cm.rainbow(np.linspace(0, 1, np.size(C_arr,1)))

    if multi:
        for i in range(len(X_arr)):
            X = X_arr[i]
            ax.plot(X[0], X[1], X[2], '.', color=col[i], ms=0.8)
    else:
        ax.plot(X_arr[0], X_arr[1], X_arr[2], '.', color='magenta', ms=0.4)
    plot_cameras_and_axes(ax, C_arr, axis_arr, s, valid_idx, col)

    ax.set_xlabel(r'$X$')
    ax.set_ylabel(r'$Y$')
    ax.set_zlabel(r'$Z$')
    ax.set_aspect('equal')
    ax.set_title(title)
    fig.tight_layout()
    plt.legend(loc="lower right")
    plt.show()


def plot_3D_points(X):
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot(X[0], X[1], X[2], '.', ms=1, color='magenta', label='X')
    ax.set_xlabel('$x$')
    ax.set_ylabel('$y$')
    ax.set_zlabel('$z$')
    ax.set_aspect('equal')
    ax.legend(loc="lower right")
    fig.tight_layout()
    plt.show()