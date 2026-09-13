import sensor,time, math, gc,struct,image
from machine import UART


# --- 1. 初始化设置 ---
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_auto_gain(False)      # 必须关闭，防止RGB数值波动
sensor.set_auto_whitebal(False)  # 必须关闭
sensor.skip_frames(time = 2000)  # 等待感光元件稳定
# 初始化串口 波特率设置为115200
uart = UART(12, baudrate=115200)     # 初始化串口 波特率设置为115200
#时钟
clock = time.clock()
# --- 2. 参数定义 ---
# 严格遵循你的行列定义：16行 x 12列
TOTAL_ROWS, TOTAL_COLS = 16, 12
LENS_STRENGTH = 1.6

# LAB 阈值定义 (替代原先的 RGB 阈值)
LAB_THRESH_WALL = (27, 100, -9, 24, -50, -4)     # 对应墙壁 (使用 WALL_LOCATOR 逻辑)
LAB_THRESH_ROAD = (31, 79, -17, 109, -118, -61)   # 对应蓝色道路
LAB_THRESH_GOAL = (51, 95, 57, 99, -78, -43)   # 对应紫色目的地

COLOR_THRESHOLDS = {
    'WALL_LOCATOR': [(27, 100, -9, 24, -50, -4)],
    'YELLOW': [(60, 100, -39, 10, 28, 91)],
    'BOMB': [(35, 71, 28, 119, -17, 71)],
    'HEAD_CYAN': [(52, 91, -61, -10, -69, -23)],
    'TAIL_GREEN': [(53, 86, -84, -19, 31, 99)],
    'GOAL_PURPLE': [(51, 95, 57, 99, -43, -78)]
}                                                   # 颜色阈值

# 滤波与状态
ALPHA_POS, ALPHA_ANG = 0.25, 0.20       # 位置和角度的滤波系数
BODY_DIST_MIN, BODY_DIST_MAX = 4, 30    # 机器人身体距离范围
s_rx, s_ry, s_ra = 0.0, 0.0, 0.0        # 初始位置和角度
last_valid_rect = (0, 0, 320, 240)       # 初始有效矩形区域

# ================= 新增：抗震动数量滤波变量 =================
stable_box_count = -1            # 系统认定的稳定箱子数量 (-1表示强制初始化)
stable_bomb_count = -1           # 系统认定的稳定炸弹数量
box_change_frames = 0            # 箱子数量异动连续帧数
bomb_change_frames = 0           # 炸弹数量异动连续帧数
CONFIRM_FRAMES = 5               # 容忍震动的连续帧数阈值
# ============================================================

last_map = None                  # 记录上一次的地图数组（展平后）
same_map_count = 0               # 连续相同地图的次数
send_flag = False                 # 发送确定标志位,上电先发地图
send_map = True                 # 发送信号标志位，要准备发送地图
TRIGGER_COUNT =  3            # 连续相同次数阈值（三次）

# --- 新增：地图锁定状态变量 ---
map_locked = False               # 大框锁定标志
lock_count = 0                   # 连续稳定帧数计数

# --- 3. 辅助判定函数 (修改为判定 LAB) ---
def is_lab_match(pixel_lab, th):
    return (th[0] <= pixel_lab[0] <= th[1] and
            th[2] <= pixel_lab[1] <= th[3] and
            th[4] <= pixel_lab[2] <= th[5])   # 判断像素是否在LAB阈值内

# --- 辅助函数：比较两个地图是否相同 ---
def is_map_same(map1, map2):
    if map2 is None:
        return False
    flat1 = [val for row in map1 for val in row]
    flat2 = [val for row in map2 for val in row]
    return flat1 == flat2

# --- 4. 实体识别函数 ---
def find_entities(img, g_rect):
    gx, gy, gw, gh = g_rect                     # 全局变量：全局矩形区域
    pw, ph = gw/TOTAL_COLS, gh/TOTAL_ROWS       # 像素宽度和高度
    boxes, bombs, goals = [], [], []                       # 初始化箱子和炸弹列表
    global s_rx, s_ry, s_ra                     # 全局变量：机器人位置和角度
    global box_count                            # 全局变量：箱子计数器
    global bomb_count                           # 全局变量：炸弹计数器
    global actual_box_count                     # 实际箱子计数器
    global actual_bomb_count                    # 实际炸弹计数器
    box_count=0
    bomb_count=0

    # A. 机器人识别
    h_blobs = img.find_blobs(COLOR_THRESHOLDS['HEAD_CYAN'], area_threshold=8)    # 查找头 blob
    t_blobs = img.find_blobs(COLOR_THRESHOLDS['TAIL_GREEN'], area_threshold=8)    # 查找尾 blob

    if h_blobs and t_blobs:                                                       # 如果找到头 blob 和尾 blob
        h = max(h_blobs, key=lambda b: b.area())    # 找到头 blob 中面积最大的
        t = max(t_blobs, key=lambda b: b.area())    # 找到尾 blob 中面积最大的

        # 【数学映射】物理坐标映射为顺时针90度虚拟系
        h_vx, h_vy = 240 - h.cy(), h.cx()
        t_vx, t_vy = 240 - t.cy(), t.cx()

        dist = math.sqrt((h_vx-t_vx)**2 + (h_vy-t_vy)**2)                 # 计算头 blob 和尾 blob 之间的距离
        if BODY_DIST_MIN < dist < BODY_DIST_MAX:                                  # 如果距离在范围内
            center_x, center_y = (h_vx + t_vx) / 2, (h_vy + t_vy) / 2     # 计算头 blob 和尾 blob 中心的坐标
            raw_rx = (center_x - gx) / pw                                                      # 计算机器人中心的x坐标
            raw_ry = (center_y - gy) / ph                                                      # 计算机器人中心的y坐标
            raw_ra = (math.degrees(math.atan2(-(h_vy-t_vy), h_vx-t_vx)) + 360) % 360  # 计算机器人角度

            s_rx = (ALPHA_POS * raw_rx) + (1 - ALPHA_POS) * s_rx                                 # 计算机器人中心的x坐标
            s_ry = (ALPHA_POS * raw_ry) + (1 - ALPHA_POS) * s_ry                                 # 计算机器人中心的y坐标

            diff = raw_ra - s_ra                                                     # 计算角度差
            if diff > 180: diff -= 360
            elif diff < -180: diff += 360
            s_ra = (s_ra + ALPHA_ANG * diff) % 360                                          # 计算机器人角度

    # B. 箱子识别 (保持 float 类型坐标)
    for b in img.find_blobs(COLOR_THRESHOLDS['YELLOW'], area_threshold=25):                   # 查找黄色 blob
        b_vx, b_vy = 240 - b.cy(), b.cx()                                                    # 【数学映射】获取虚拟中心
        bx, by = (b_vx-gx)/pw, (b_vy-gy)/ph                                              # 计算机器人中心的xy坐标
        if 0 <= bx < TOTAL_COLS and 0 <= by < TOTAL_ROWS:                                     # 如果箱子在地图范围内
            boxes.append((round(float(bx), 2), round(float(by), 2)))                         # 加入箱子列表
            # img.draw_cross(b.cx(), b.cy(), color=(255, 255, 0), size=6)                        # 绘制黄色 blob 中心的十字线 (使用真实坐标系不变)
            if bx > 0 and by > 0:
                box_count += 1

    # C. 炸弹识别 (保持 float 类型坐标)
    for b in img.find_blobs(COLOR_THRESHOLDS['BOMB'], area_threshold=25):
        b_vx, b_vy = 240 - b.cy(), b.cx()                                                    # 【数学映射】获取虚拟中心
        bmx, bmy = (b_vx-gx)/pw, (b_vy-gy)/ph
        if 0 <= bmx < TOTAL_COLS and 0 <= bmy < TOTAL_ROWS:
            bombs.append((round(float(bmx), 2), round(float(bmy), 2)))
            # img.draw_cross(b.cx(), b.cy(), color=(255, 0, 255), size=0)                        # 绘制黄色 blob 中心的十字线
            if bmx > 0 and bmy > 0:
                bomb_count += 1

    # D. 紫色目的地识别
    for b in img.find_blobs(COLOR_THRESHOLDS['GOAL_PURPLE'], area_threshold=25):
            # 1. 获取色块的物理边界框
            rect = b.rect()

            # 2. 纯数学计算：【映射为虚拟边界框】，反推这个大色块覆盖的候选网格范围 (划定嫌疑区)
            bx, by, bw, bh = 240 - rect[1] - rect[3], rect[0], rect[3], rect[2]

            col_start = int((bx - gx) / pw)
            col_end   = int((bx + bw - gx) / pw)
            row_start = int((by - gy) / ph)
            row_end   = int((by + bh - gy) / ph)

            # 3. 边界保护
            col_start = max(0, min(col_start, TOTAL_COLS - 1))
            col_end   = max(0, min(col_end, TOTAL_COLS - 1))
            row_start = max(0, min(row_start, TOTAL_ROWS - 1))
            row_end   = max(0, min(row_end, TOTAL_ROWS - 1))

            # 4. 遍历嫌疑区内的所有网格，进行 LAB 精准检验
            for r in range(row_start, row_end + 1):
                for c in range(col_start, col_end + 1):

                    # 计算当前候选网格的虚拟绝对中心
                    grid_cx = int(gx + (c + 0.5) * pw)
                    grid_cy = int(gy + (r + 0.5) * ph)

                    # 在中心进行 3x3 像素采样 (虚拟系)
                    points = [(grid_cx, grid_cy), (grid_cx-1, grid_cy-1), (grid_cx, grid_cy-1), (grid_cx+1, grid_cy-1),
                              (grid_cx-1, grid_cy),                       (grid_cx+1, grid_cy),
                              (grid_cx-1, grid_cy+1), (grid_cx, grid_cy+1), (grid_cx+1, grid_cy+1)]

                    r_sum = g_sum = b_sum = v_cnt = 0
                    for px_v, py_v in points:
                        # 【反向映射】从虚拟系反向推算回真实的物理像素位置
                        orig_x = py_v
                        orig_y = 239 - px_v

                        if 0 <= orig_x < 320 and 0 <= orig_y < 240:
                            pix = img.get_pixel(orig_x, orig_y)
                            r_sum += pix[0]; g_sum += pix[1]; b_sum += pix[2]; v_cnt += 1

                    if v_cnt > 0:
                        p_avg_rgb = (r_sum // v_cnt, g_sum // v_cnt, b_sum // v_cnt)
                        p_avg_lab = image.rgb_to_lab(p_avg_rgb)

                        # 5. 只有真正是紫色的格子，才会被认作目的地
                        if is_lab_match(p_avg_lab, LAB_THRESH_GOAL):
                            goal_coord = (round(float(c)+0.5, 2), round(float(r)+0.5, 2))

                            if goal_coord not in goals:
                                goals.append(goal_coord)
                                # 在画面上画一个十字，精准标记识别到的中心位置
                                # img.draw_cross(int(320 - grid_cy), int(grid_cx), color=(128, 0, 128), size=5)             # 绘制紫色十字线（区分目的地）

    return s_rx, s_ry, s_ra, boxes, bombs, goals

# --- 6. 数据打包函数 ---
def adaptive_send(uart, img, g_rect, grid_map):

    global stable_box_count, stable_bomb_count, box_change_frames, bomb_change_frames
    global last_map, same_map_count, send_flag, send_map, TRIGGER_COUNT

    rx, ry, ra, boxes, bombs, goals = find_entities(img, g_rect)
    current_box_count = actual_box_count
    current_bomb_count = actual_bomb_count

    # =================== 核心抗震动防抖逻辑 ===================
    # 1. 初始化阶段：如果稳定数量还是 -1，强制直接更新，完成开机第一拍
    if stable_box_count == -1:
        stable_box_count = current_box_count
        stable_bomb_count = current_bomb_count
        send_map = True

    else:
        # 2. 检测箱子数量异动
        if current_box_count != stable_box_count:
            box_change_frames += 1
            if box_change_frames >= CONFIRM_FRAMES:  # 连续 5 帧都变了，认定为长久性改变
                stable_box_count = current_box_count # 接受新的现实
                box_change_frames = 0
                send_map = True                      # 触发地图更新
        else:
            box_change_frames = 0                    # 一旦恢复正常，计数器清零，说明刚才只是震动假象

        # 3. 检测炸弹数量异动
        if current_bomb_count != stable_bomb_count:
            bomb_change_frames += 1
            if bomb_change_frames >= CONFIRM_FRAMES: # 连续 5 帧都变了，认定为长久性改变
                stable_bomb_count = current_bomb_count
                bomb_change_frames = 0
                send_map = True                      # 触发地图更新
        else:
            bomb_change_frames = 0
    # ============================================================

    if send_map == True:
        if is_map_same(grid_map, last_map):
            same_map_count += 1  # 连续相同次数+1
        else:
            same_map_count = 0   # 地图变化，重置计数

    if same_map_count == TRIGGER_COUNT:  #检测到连续三次相同
        send_flag = True

    if send_flag:
        flat_map = [int(val) for row in grid_map for val in row]
        all_data = [0xA6] + flat_map + [0x6A]
        packet = struct.pack('<B192hB', *all_data)   #194字节
        uart.write(packet)
        send_map = False
        send_flag = False
        same_map_count =0
        #打印地图
        print("--- Map Matrix (16x12) ---")
        for row in matrix:
            print(" ".join(map(str, row)))
        print("========================================")
    else:
        if send_map == False:    #等待地图发送成功
            cx, cy = float(rx), float(ry)
            bomb_data = []
            for i in range(3):
                if i < len(bombs):
                    bomb_data.append(float(bombs[i][0]))
                    bomb_data.append(float(bombs[i][1]))
                else:
                    bomb_data.append(0.0)
                    bomb_data.append(0.0)

            box_data = []
            for i in range(3):
                if i < len(boxes):
                    box_data.append(float(boxes[i][0]))
                    box_data.append(float(boxes[i][1]))
                else:
                    box_data.append(0.0)
                    box_data.append(0.0)

            goal_data = []
            for i in range(3):
                if i < len(goals):
                    goal_data.append(float(goals[i][0]))
                    goal_data.append(float(goals[i][1]))
                else:
                    goal_data.append(0.0)
                    goal_data.append(0.0)

            all_data = [0xA5] + [cx, cy] + bomb_data + box_data + goal_data + [0x5A]
            packet = struct.pack('<B20fB', *all_data)  #82字节
            uart.write(packet)

    last_map = [row.copy() for row in grid_map]

# --- 5. 主循环 ---
while(True):
    if (clock.fps() % 10) == 0:  # 每10帧进行一次垃圾回收
        gc.collect()
    img = sensor.snapshot()                         # 拍摄图像
    #img.lens_corr(strength = LENS_STRENGTH)         # 畸变校正
    #img.rotation_corr(z_rotation=90.0)              # 图片旋转

    # A. 第一步：定位地图大框
    if not map_locked:    # <--- 【新增】锁定判断，锁定后跳过耗时的大框搜索
        # 【数学映射】将你想在虚拟系统中搜索的 ROI (0, 0, 180, 220) 映射回真实的物理图像区域
        search_roi = (30,30,255,185)
        blobs = img.find_blobs(COLOR_THRESHOLDS['WALL_LOCATOR'],
                                   roi=search_roi,
                                   area_threshold=150, # 稍微降低，允许细线参与
                                   merge=True,
                                   margin=60)          # 减小 margin 防止粘连背景

        if blobs:
            # 核心逻辑：在所有 blob 中，寻找最像“框”的那一个
            valid_blobs = [b for b in blobs if 50 < b.w() < 320 and 50 < b.h() < 240]

            if valid_blobs:
                # 筛选逻辑：选择面积最大且最接近正方形/目标比例的 (由于虚拟了90度旋转，真实图像评估时长宽比也要翻过来：b.h()/b.w() )
                target_blob = max(valid_blobs, key=lambda b: b.area() / (abs(0.65 - b.h()/b.w()) + 0.1))

                # 只有当这个 blob 真的够大时才进入滤波逻辑
                if target_blob.area() > 500:
                    # --- 修改部分：增强稳定性（惯性滤波） ---
                    rect = target_blob.rect()

                    # 【数学映射】将物理提取的大框直接映射为“虚拟旋转90度”后的大框
                    new_rect = (240 - rect[1] - rect[3], rect[0], rect[3], rect[2])

                    # 使用 ALPHA_POS (0.25) 或自定义系数进行平滑，防止断层导致的瞬时跳变
                    # 计算公式：新值 * 0.3 + 旧值 * 0.7
                    xin = 0.7
                    jiu = 0.3
                    smooth_x = int(new_rect[0] * xin + last_valid_rect[0] * jiu)
                    smooth_y = int(new_rect[1] * xin + last_valid_rect[1] * jiu)
                    smooth_w = int(new_rect[2] * xin + last_valid_rect[2] * jiu)
                    smooth_h = int(new_rect[3] * xin + last_valid_rect[3] * jiu)

                    last_valid_rect = (smooth_x, smooth_y, smooth_w, smooth_h)
                    # --- 修改结束 ---

                    lock_count += 1
                    if lock_count > 15:  # 连续稳定 15 帧后锁定
                        map_locked = True
        else:
            lock_count = 0  # 若丢失边框则重置计数

    gx, gy, gw, gh = last_valid_rect
    cw, ch = gw/TOTAL_COLS, gh/TOTAL_ROWS        # 像素宽度和高度

    # 【反向映射】绘制大框时，将虚拟计算出的安全坐标反算回屏幕进行显示
    img.draw_rectangle((gy, 240 - gx - gw, gh, gw), color=(0, 0, 0), thickness=2) # 绘制地图大框矩形
    img.draw_rectangle((30,30,255,185),color=(0, 0, 220),thickness=3)

    # B. 第二步：实体识别
    rx, ry, ra, box_list, bomb_list, goal_list = find_entities(img, last_valid_rect)

    # 建立排除禁区 (修改为分类映射为整数索引)
    box_cells = set()
    for b in box_list: box_cells.add((int(b[0]), int(b[1])))

    bomb_cells = set()
    for b in bomb_list: bomb_cells.add((int(b[0]), int(b[1])))

    robot_cell = (int(rx), int(ry))

    # C. 第三步：网格扫描 (完全切换为 LAB 识别)
    matrix = [[0 for _ in range(TOTAL_COLS)] for _ in range(TOTAL_ROWS)]   # 初始化地图矩阵
    for r in range(TOTAL_ROWS):
        for c in range(TOTAL_COLS):

            # --- 优先将箱子和炸弹的初始坐标标记进地图 ---
            if (c, r) in box_cells:
                matrix[r][c] = 3      # 箱子所在格子设为 3
                continue

            if (c, r) in bomb_cells:
                matrix[r][c] = 6      # 炸弹所在格子设为 6
                continue

            if (c, r) == robot_cell:  # 注意必须用 == 判断元组相等
                matrix[r][c] = 0      # 机器人底盘算作道路 (0)
                continue
            # ----------------------------------------------

            if r == 0 or r == TOTAL_ROWS - 1 or c == 0 or c == TOTAL_COLS - 1:
                matrix[r][c] = 2
                continue

            cx, cy = int(gx + (c+0.5)*cw), int(gy + (r+0.5)*ch)   # 计算虚拟中心坐标
            points = [(cx, cy), (cx-2, cy-2), (cx+2, cy-2), (cx-2, cy+2), (cx+2, cy+2)
                      ,(cx-1, cy-1), (cx+1, cy-1), (cx-1, cy+1), (cx+1, cy+1)]
            r_sum = g_sum = b_sum = v_cnt = 0

            for px_v, py_v in points:
                # 【反向映射】将算出来的虚拟网格像素点退回真实的横屏坐标系中抓取颜色！
                orig_x = py_v
                orig_y = 239 - px_v
                if 0 <= orig_x < 320 and 0 <= orig_y < 240:
                    pix = img.get_pixel(orig_x, orig_y)
                    r_sum += pix[0]; g_sum += pix[1]; b_sum += pix[2]; v_cnt += 1

            if v_cnt > 0:
                p_avg_rgb = (r_sum // v_cnt, g_sum // v_cnt, b_sum // v_cnt)
                # 核心改变：RGB 转 LAB
                p_avg_lab = image.rgb_to_lab(p_avg_rgb)

                if is_lab_match(p_avg_lab, LAB_THRESH_GOAL):
                    matrix[r][c] = 4
                elif is_lab_match(p_avg_lab, LAB_THRESH_WALL):
                    matrix[r][c] = 2
                elif is_lab_match(p_avg_lab, LAB_THRESH_ROAD):
                    matrix[r][c] = 0
                # else:
                #     # 兜底逻辑同样基于 LAB 的 B 通道 (蓝黄色轴)，B越小越偏蓝
                #     matrix[r][c] = 0 if p_avg_lab[2] < -10 else 2

    actual_box_count = box_count
    actual_bomb_count = bomb_count

    adaptive_send(uart, img, last_valid_rect, matrix)



    # print("Robot Pos: (%.2f, %.2f)" % (rx, ry))

    # if box_list:
    #     print("Boxes found at:", ["(%.2f, %.2f)" % (b[0], b[1]) for b in box_list])
    # else:
    #     print("No Boxes detected.")
    # if bomb_list:
    #         print("Bombs found at:", ["(%.2f, %.2f)" % (b[0], b[1]) for b in bomb_list])
    # else:
    #     print("No Bombs detected.")

    # print("-" * 30) # 分割线
    # if goal_list:
    #         print("找到 %d 个目的地, 坐标:" % len(goal_list), ["(%.2f, %.2f)" % (g[0], g[1]) for g in goal_list])
    # else:
    #         print("未检测到目的地。")
    time.sleep_ms(3)
