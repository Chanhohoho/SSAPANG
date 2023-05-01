import os
import shutil
import csv

current_path = os.getcwd()
folder_list = os.listdir(current_path)
# num = 0
# for folder in folder_list:materials
   
# #     model_name = folder
# #     print(f'<model name="{model_name}">\n' \
# #         '\t<include>\n' \
# #         f'\t\t<uri>model://qrcode/{model_name}</uri>\n' \
# #         '\t</include>\n' \
# #         '\t<pose frame="">0 0 0.01 0 0 0</pose>\n' \
# #         '</model>')




for folder_name in folder_list:
    if folder_name == 'worldfile.py' or folder_name =='qrcodes' or folder_name == 'mkdir.py':
        continue
    print(folder_name)
    
    current_path1 = f'{current_path}/{folder_name}/materials'
    os.mkdir(f'{current_path1}')
#     print(current_path1)
#     if not os.listdir(f'{current_path}/{folder_name}/materials'):
        
#         os.mkdir(f'{current_path1}/scripts')
#         os.mkdir(f'{current_path1}/textures')
        
#         #material 파일 생성
#         with open(f'{current_path1}/scripts/{folder_name}.material', 'w') as f:
#             f.write(f"material {folder_name}\n" \
#             f"{{\n" \
#             f"  technique\n" \
#             f"  {{\n" \
#             f"    pass\n" \
#             f"    {{\n" \
#             f"      texture_unit\n" \
#             f"      {{\n" \
#             f"        texture {folder_name}.png\n" \
#             f"      }}\n" \
#             f"    }}\n" \
#             f"  }}\n" \
#             f"}}")
        
#         with open(f'{current_path}/{folder_name}/model.sdf', 'w') as f:
#             f.write(
#         f'<?xml version="1.0" ?>\n' \
#         f'<sdf version="1.6">\n' \
#         f'  <model name="{folder_name}">\n' \
#         f'    <static>true</static>\n' \
#         f'    <link name="{folder_name}">\n' \
#         f'      <visual name="qr_visual">\n' \
#         f'        <cast_shadows>false</cast_shadows>\n' \
#         f'        <geometry>\n' \
#         f'          <plane>\n' \
#         f'            <normal>0 0 1</normal>\n' \
#         f'            <size>0.1 0.1</size>\n' \
#         f'          </plane>\n' \
#         f'        </geometry>\n' \
#         f'        <material>\n' \
#         f'          <script>\n' \
#         f'            <!--uri>file://materials/scripts/{folder_name}.material</uri-->\n' \
#         f'            <uri>model://qrcode/{folder_name}/materials/scripts</uri>\n' \
#         f'            <uri>model://qrcode/{folder_name}/materials/textures</uri>\n' \
#         f'            <name>{folder_name}</name>\n' \
#         f'          </script>\n' \
#         f'        </material>\n' \
#         f'      </visual>\n' \
#         f'    </link>\n' \
#         f'  </model>\n' \
#         f'</sdf>')
        
#         with open(f'{current_path}/{folder_name}/model.config', 'w') as f:
#             f.write(f'<?xml version="1.0" ?>\n'
#             f'<model>\n'
#             f'    <name>{folder_name}</name>\n'
#             f'    <version>1.0</version>\n'
#             f'    <sdf version="1.6">model.sdf</sdf>\n'
#             f'    <author>\n'
#             f'        <name></name>\n'
#             f'        <email></email>\n'
#             f'    </author>\n'
#             f'    <description></description>\n'
#             f'</model>')    



        

#     else:
#         print(current_path1)
    
    
