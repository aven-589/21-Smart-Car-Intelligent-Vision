import pyb
import sensor, image, time, math
import os, tf
import struct
from machine import UART

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA) # we run out of memory if the resolution is much bigger...
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False)  # must turn this off to prevent image washout...
sensor.set_auto_whitebal(False)  # must turn this off to prevent image washout...
clock = time.clock()
uart = UART(12, baudrate=115200)# 初始化串口 波特率设置为115200

def find_existing_path(path_list):
 for path in path_list:
     try:
         os.stat(path)
         return path
     except OSError:
         pass
 return None

# 模型文件路径
person_model_name = "mobilenet_v3.tflite"
person_label_name = "model.txt"
# 两个数字模型：v3 和 v4
num_model_name_v3 = "mobilenet_v3-num.tflite"
num_model_name_v4 = "mobilenet_v4-num.tflite"
num_label_name = "num.txt"
# 用于对大头儿子/喜羊羊进行二次确认的模型
confirm_model_name = "mobilenet_v4-dlc.tflite"
confirm_label_name = "model_dlc.txt"

num_model_path_v3 = find_existing_path((num_model_name_v3, "/sd/" + num_model_name_v3, "/flash/" + num_model_name_v3))
num_model_path_v4 = find_existing_path((num_model_name_v4, "/sd/" + num_model_name_v4, "/flash/" + num_model_name_v4))
num_label_path    = find_existing_path((num_label_name, "/sd/" + num_label_name, "/flash/" + num_label_name))
person_model_path = find_existing_path((person_model_name, "/sd/" + person_model_name, "/flash/" + person_model_name))
person_label_path = find_existing_path((person_label_name, "/sd/" + person_label_name, "/flash/" + person_label_name))
confirm_model_path = find_existing_path((confirm_model_name, "/sd/" + confirm_model_name, "/flash/" + confirm_model_name))
confirm_label_path = find_existing_path((confirm_label_name, "/sd/" + confirm_label_name, "/flash/" + confirm_label_name))

if person_model_path is None:
 raise OSError("model file not found: " + person_model_name)
if person_label_path is None:
 raise OSError("label file not found: " + person_label_name)
# 至少需要一个数字模型存在
if num_model_path_v3 is None and num_model_path_v4 is None:
 raise OSError("no number model found: " + num_model_name_v3 + " or " + num_model_name_v4)
if num_label_path is None:
 raise OSError("label file not found: " + num_label_name)

person_labels = [line.rstrip() for line in open(person_label_path)]
num_labels = [line.rstrip() for line in open(num_label_path)]

net_confirm = None
confirm_labels = []
if confirm_model_path is not None and confirm_label_path is not None:
 confirm_labels = [line.rstrip() for line in open(confirm_label_path)]
 try:
     net_confirm = tf.load(confirm_model_path, load_to_fb=True)
     print("Confirmation model loaded successfully")
 except OSError:
     print("load_to_fb=True failed for confirmation model, retrying without framebuffer")
     net_confirm = tf.load(confirm_model_path, load_to_fb=False)
else:
 print("Confirmation model not found, skipping secondary confirmation")

# 加载模型
try:
 net_person = tf.load(person_model_path, load_to_fb=True)
 print("Person model loaded successfully")
except OSError:
 print("load_to_fb=True failed for person model, retrying without framebuffer")
 net_person = tf.load(person_model_path, load_to_fb=False)

# 支持两个数字模型（v3 / v4），至少加载成功一个
net_num_v3 = None
net_num_v4 = None

if num_model_path_v3 is not None:
 try:
     net_num_v3 = tf.load(num_model_path_v3, load_to_fb=True)
     print("Number model v3 loaded successfully")
 except OSError:
     print("load_to_fb=True failed for number model v3, retrying without framebuffer")
     net_num_v3 = tf.load(num_model_path_v3, load_to_fb=False)

if num_model_path_v4 is not None:
 try:
     net_num_v4 = tf.load(num_model_path_v4, load_to_fb=True)
     print("Number model v4 loaded successfully")
 except OSError:
     print("load_to_fb=True failed for number model v4, retrying without framebuffer")
     net_num_v4 = tf.load(num_model_path_v4, load_to_fb=False)

# 按照用户定义的编号映射（直接按照你发的#define来）
# model.txt中的索引顺序：0:big_head_son, 1:calabash_brothers, 2:donald_duck, 3:gg_bond, 4:grey_wolf,
#                       5:mickey_mouse, 6:nezha, 7:pikachu, 8:pleasant_sheep, 9:spongebob_squarepants
# 你定义的发送编号：
# #define mickey_mouse             0
# #define pikachu                  1
# #define spongebob_squarepants    2
# #define pleasant_sheep           3
# #define donald_duck              4
# #define nezha                    5
# #define big_head_son             6
# #define gg_bond                  7
# #define calabash_brothers        8
# #define grey_wolf                9

# 模型输出索引 -> 发送编号
person_id_map = {
 0: 6,  # big_head_son -> 发送6
 1: 8,  # calabash_brothers -> 发送8
 2: 4,  # donald_duck -> 发送4
 3: 7,  # gg_bond -> 发送7
 4: 9,  # grey_wolf -> 发送9
 5: 0,  # mickey_mouse -> 发送0
 6: 5,  # nezha -> 发送5
 7: 1,  # pikachu -> 发送1
 8: 3,  # pleasant_sheep -> 发送3
 9: 2,  # spongebob_squarepants -> 发送2
}

# 识别模式：0=未设置，1=识别人物，2=识别数字
recognition_mode = 0

# 发送人物数据包：帧头0x5A + 人物编号 + 帧尾0xA5
def send_person_data(person_id):
 all_data = [0x5A, person_id, 0xA5]
 packet = struct.pack('<BBB', *all_data)  # 3字节
 uart.write(packet)
 print(f"Sent person data: 0x5A, {person_id}, 0xA5")

# 发送数字数据包：帧头0x6A + 数字 + 帧尾0xA6
def send_number_data(number):
 all_data = [0x6A, number, 0xA6]
 packet = struct.pack('<BBB', *all_data)  # 3字节
 uart.write(packet)
 print(f"Sent number data: 0x6A, {number}, 0xA6")

# 用于连续三次相同识别判断的状态
person_last_id = None
person_count = 0
number_last = None
number_count = 0
# 固定识别区域，移动 ROI: 增加 ROI_X 向右移动，减少 ROI_X 向左；增加 ROI_Y 向下移动，减少 ROI_Y 向上。
# 调整大小: 增大 ROI_W/ROI_H 包含更大目标，减小则只关注局部细节。
ROI_X = 58
ROI_Y = 65
ROI_W = 220
ROI_H = 162

# 屏幕中间光斑抑制圆圈配置
CIRCLE_CENTER_X = 160  # 默认画面中心 x
CIRCLE_CENTER_Y = 120  # 默认画面中心 y
CIRCLE_RADIUS = 80     # 可调整半径
CIRCLE_DARKEN_FACTOR = 0.55  # 亮度缩放因子，0.0~1.0

# 将指定圆圈区域亮度降低，减弱中央光斑影响
# def darken_circle_region(img, center_x, center_y, radius, factor):
#     if factor >= 1.0 or radius <= 0:
#         return
#     width = img.width()
#     height = img.height()
#     min_x = max(center_x - radius, 0)
#     max_x = min(center_x + radius, width - 1)
#     min_y = max(center_y - radius, 0)
#     max_y = min(center_y + radius, height - 1)
#     radius_sq = radius * radius
#     for y in range(min_y, max_y + 1):
#         dy = y - center_y
#         dy_sq = dy * dy
#         for x in range(min_x, max_x + 1):
#             dx = x - center_x
#             if dx * dx + dy_sq <= radius_sq:
#                 pixel = img.get_pixel(x, y)
#                 if pixel is None:
#                     continue
#                 r = int(pixel[0] * factor)
#                 g = int(pixel[1] * factor)
#                 b = int(pixel[2] * factor)
#                 img.set_pixel(x, y, (r, g, b))

# （已移除）LAB 阈值相关函数

print("Waiting for UART command: 0x01=person, 0x02=number")

while(True):
 img = sensor.snapshot()
 img.rotation_corr(z_rotation=180.0)
 # darken_circle_region(img, CIRCLE_CENTER_X, CIRCLE_CENTER_Y, CIRCLE_RADIUS, CIRCLE_DARKEN_FACTOR)
 roi = (ROI_X, ROI_Y, ROI_W, ROI_H)
 img.draw_rectangle(roi, color = (255, 0, 0))
 roi_img = img.copy(1, 1, roi)
 recognition_mode = 2  # 每帧重置模式，等待新的UART指令设置模式
 #检查串口指令
 uart_num = uart.any()
 if uart_num:
     cmd = uart.read(1)  # 读取1字节指令
     if cmd and len(cmd) > 0:
         if cmd[0] == 0x01:
             recognition_mode = 1
             print("Mode: Person Recognition")
         elif cmd[0] == 0x02:
             recognition_mode = 2
             print("Mode: Number Recognition")

 # 根据模式执行识别
 if recognition_mode == 1:  # 识别人物
     detected = False
     for obj in tf.classify(net_person, roi_img, min_scale=1.0, scale_mul=0.5, x_overlap=0.0, y_overlap=0.0):
         sorted_list = sorted(zip(person_labels, obj.output()), key = lambda x: x[1], reverse = True)
         top_label, top_score = sorted_list[0]
         if top_score > 0.60:
             label_idx = person_labels.index(top_label)
             person_id = person_id_map.get(label_idx, 0xFF)
             confirmed = True

             # 对大头儿子/喜羊羊做二次确认
             if top_label in ("big_head_son", "pleasant_sheep") and net_confirm is not None:
                 for confirm_obj in tf.classify(net_confirm, roi_img, min_scale=1.0, scale_mul=0.5, x_overlap=0.0, y_overlap=0.0):
                     confirm_sorted_list = sorted(zip(confirm_labels, confirm_obj.output()), key=lambda x: x[1], reverse=True)
                     confirm_label, confirm_score = confirm_sorted_list[0]
                     print(f"Confirm {top_label} -> {confirm_label} = {confirm_score:.2f}")
                     if confirm_score > 0.60 and confirm_label == top_label:
                         confirmed = True
                     else:
                         confirmed = False
                     break

             if confirmed:
                 print(f"{top_label} = {top_score:.2f}, ID={person_id}")
                 detected = True
                 # 判断是否与上一次相同
                 if person_last_id is None or person_last_id != person_id:
                     person_last_id = person_id
                     person_count = 1
                 else:
                     person_count += 1

                 # 连续三次相同则发送
                 if person_count >= 3:
                     send_person_data(person_id)
                     # 发送后重置计数，需要再连续三帧才能再次发送
                     person_count = 0
             else:
                 print(f"Secondary confirmation failed for {top_label}")
             break

     # 如果本帧没有检测到（或置信度不够），则重置计数
     if not detected:
         person_last_id = None
         person_count = 0

 elif recognition_mode == 2:  # 识别数字
     results = []  # tuples of (model_tag, top_label, top_score)

     if net_num_v3 is not None:
         for obj in tf.classify(net_num_v3, roi_img, min_scale=1.0, scale_mul=0.5, x_overlap=0.0, y_overlap=0.0):
             sorted_list = sorted(zip(num_labels, obj.output()), key=lambda x: x[1], reverse=True)
             top_label_v3, top_score_v3 = sorted_list[0]
             results.append(('v3', top_label_v3, top_score_v3))
             break

     if net_num_v4 is not None:
         for obj in tf.classify(net_num_v4, roi_img, min_scale=1.0, scale_mul=0.5, x_overlap=0.0, y_overlap=0.0):
             sorted_list = sorted(zip(num_labels, obj.output()), key=lambda x: x[1], reverse=True)
             top_label_v4, top_score_v4 = sorted_list[0]
             results.append(('v4', top_label_v4, top_score_v4))
             break

     # 过滤出置信度大于0.5的结果
     good = [r for r in results if r[2] > 0.5]
     chosen_number = None
     chosen_score = 0.0
     chosen_tag = None

     if len(good) == 1:
         chosen_tag, top_label, chosen_score = good[0]
         chosen_number = int(top_label)
         print(f"Number {top_label} = {chosen_score:.2f} (from {chosen_tag})")
     elif len(good) > 1:
         best = max(good, key=lambda x: x[2])
         # 打印两个模型的分数与最终选择
         v3_score = 0.0
         for r in results:
             if r[0] == 'v3':
                 v3_score = r[2]
                 break
         v4_score = 0.0
         for r in results:
             if r[0] == 'v4':
                 v4_score = r[2]
                 break
         print(f"Model scores: v3={v3_score:.2f}, v4={v4_score:.2f}")
         chosen_tag, top_label, chosen_score = best
         chosen_number = int(top_label)
         print(f"Chosen Number {top_label} = {chosen_score:.2f} (from {chosen_tag})")

     # 根据 chosen_number 做连续三次判断：三次相同才发送
     if chosen_number is None:
         number_last = None
         number_count = 0
     else:
         if number_last is None or number_last != chosen_number:
             number_last = chosen_number
             number_count = 1
         else:
             number_count += 1

         if number_count >= 1:
             send_number_data(chosen_number)
             # 发送后重置计数，需要再连续三帧才能再次发送
             number_count = 0

