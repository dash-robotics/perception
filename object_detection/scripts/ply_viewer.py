import numpy as np
from open3d import *    

def main():
    pcd = read_point_cloud('../templates/merged_cloud.ply')
    draw_geometries([pcd])

if __name__ == "__main__":
    main()
