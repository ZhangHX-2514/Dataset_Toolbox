import numpy as np
from glob import glob
import h5py
import os

from src.visualize import vis_utils as vis

from src.io.psee_loader import PSEELoader

# if __name__ == "__main__":
#     dat_file_path = '.\\17-10-11_13-38-56_122500000_182500000_td.dat'
    
#     dat = PSEELoader(dat_file_path)
#     eve = dat.load_n_events(dat._ev_count)
#     h5_file_path = dat_file_path.rstrip('.dat') + '.h5'
#     print(h5_file_path)
#     h5 = h5py.File(h5_file_path, 'w')
#     h5.create_dataset('events', data=eve)

if __name__ == "__main__":
    dat_folder_path = 'E:\\BaiduNetdiskDownload\\dataset\\val'
    output_folder_path = dat_folder_path
    
    # 获取文件夹中所有 .dat 文件
    dat_files = glob(os.path.join(dat_folder_path, '*.dat'))
    
    for dat_file in dat_files:
        dat = PSEELoader(dat_file)
        # eve = dat.load_n_events(dat._ev_count)
        eve = dat.load_n_events(10000000)
        # 生成 .h5 文件路径
        h5_file_name = os.path.basename(dat_file) + '.h5'
        h5_file_path = os.path.join(output_folder_path, h5_file_name)
        
        # 保存 .h5 文件
        with h5py.File(h5_file_path, 'w') as h5:
            h5.create_dataset('events', data=eve)

    print("Done!")