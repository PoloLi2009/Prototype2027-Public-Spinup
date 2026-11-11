"""Offline seeded ball tracking and conservative shot diagnostics."""
import math
import cv2
import numpy as np

COLORS = [(50, 180, 255), (80, 220, 90), (220, 90, 220), (255, 190, 50)]


def read_frame(path, seconds):
    cap = cv2.VideoCapture(str(path))
    try:
        if not cap.isOpened():
            raise ValueError('视频无法解码，请转成 H.264 MP4 后重试。')
        cap.set(cv2.CAP_PROP_POS_MSEC, max(0, seconds) * 1000)
        ok, frame = cap.read()
        if not ok:
            raise ValueError('该时间没有可读取的视频帧。')
        return frame
    finally:
        cap.release()


def track(path, start, end, seed_x, seed_y, radius, search=100, threshold=.55):
    """One user-selected flight window per ball; never invent missing positions."""
    if end <= start or start < 0 or radius < 3:
        raise ValueError('检查每球起止时间和球半径。')
    cap = cv2.VideoCapture(str(path))
    try:
        fps = cap.get(cv2.CAP_PROP_FPS)
        if not cap.isOpened() or not math.isfinite(fps) or fps <= 0:
            raise ValueError('视频帧率无效。')
        count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        first, last = int(round(start * fps)), min(int(end * fps), count - 1)
        if first >= last:
            raise ValueError('跟踪区间不足两帧或超出视频长度。')
        cap.set(cv2.CAP_PROP_POS_FRAMES, first)
        ok, frame = cap.read()
        if not ok:
            raise ValueError('无法读取起始帧。')
        height, width = frame.shape[:2]
        x, y, r = int(seed_x), int(seed_y), int(radius)
        if x-r < 0 or y-r < 0 or x+r+1 > width or y+r+1 > height:
            raise ValueError('球的初始框必须完整位于画面内。')
        template = cv2.cvtColor(frame[y-r:y+r+1, x-r:x+r+1], cv2.COLOR_BGR2GRAY)
        if np.std(template) < 4:
            raise ValueError('初始框缺少可辨认的纹理或边缘，请扩大半径以包含球边缘。')
        points = [{'frame': first, 't': first/fps, 'x': float(x), 'y': float(y), 'score': 1.0}]
        velocity = np.zeros(2)
        previous = np.array([x, y], dtype=float)
        missed = 0
        for index in range(first+1, last+1):
            ok, frame = cap.read()
            if not ok:
                break
            gap = index - points[-1]['frame']
            predicted = previous + velocity * gap
            margin = search * (1 + .2 * missed)
            left, top = max(0, int(predicted[0]-margin)), max(0, int(predicted[1]-margin))
            right, bottom = min(width, int(predicted[0]+margin)), min(height, int(predicted[1]+margin))
            if right-left < template.shape[1] or bottom-top < template.shape[0]:
                break
            gray = cv2.cvtColor(frame[top:bottom, left:right], cv2.COLOR_BGR2GRAY)
            result = cv2.matchTemplate(gray, template, cv2.TM_SQDIFF_NORMED)
            yy, xx = np.indices(result.shape)
            distance = np.hypot(xx+left+r-predicted[0], yy+top+r-predicted[1])
            cost = result + .08 * distance / max(search, 1)
            by, bx = np.unravel_index(np.argmin(cost), cost.shape)
            score = 1-float(result[by, bx])
            if score < threshold:
                missed += 1
                if missed >= 5:
                    break
                continue
            current = np.array([left+bx+r, top+by+r], dtype=float)
            velocity = (current-previous)/gap
            previous = current
            points.append({'frame': index, 't': index/fps, 'x': float(current[0]),
                           'y': float(current[1]), 'score': score})
            missed = 0
        return points, fps
    finally:
        cap.release()


def fit_trajectory(points, meters_per_pixel=None):
    if len(points) < 6:
        return {'usable': False, 'reason': '有效跟踪点不足 6 个，需人工补点或调整起点。'}
    values = np.array([[p['t'], p['x'], -p['y']] for p in points], dtype=float)
    if not np.isfinite(values).all() or np.any(np.diff(values[:, 0]) <= 0):
        return {'usable': False, 'reason': '时间必须递增，坐标必须有限。'}
    t = values[:, 0] - values[0, 0]
    if t[-1] < .1:
        return {'usable': False, 'reason': '可见轨迹不足 0.1 秒。'}
    keep = np.ones(len(t), dtype=bool)
    for _ in range(3):
        if keep.sum() < 6:
            return {'usable': False, 'reason': '剔除异常点后数据不足。'}
        cx = np.polyfit(t[keep], values[keep, 1], 1)
        cy = np.polyfit(t[keep], values[keep, 2], 2)
        residual = np.hypot(np.polyval(cx,t)-values[:,1],np.polyval(cy,t)-values[:,2])
        median = np.median(residual[keep])
        mad = np.median(np.abs(residual[keep]-median))
        keep = residual <= max(3, median+3*1.4826*mad)
    # Refit after the final rejection pass.
    if keep.sum() < 6:
        return {'usable': False, 'reason': '有效点不足。'}
    cx=np.polyfit(t[keep],values[keep,1],1);cy=np.polyfit(t[keep],values[keep,2],2)
    rms=float(np.sqrt(np.mean((np.polyval(cx,t[keep])-values[keep,1])**2+
                              (np.polyval(cy,t[keep])-values[keep,2])**2)))
    scale = float(meters_per_pixel) if meters_per_pixel else None
    if scale is not None and (not math.isfinite(scale) or scale <= 0):
        raise ValueError('标定比例必须大于零。')
    return {'usable': True, 'cx': cx.tolist(), 'cy': cy.tolist(), 't0': float(values[0,0]),
            'duration_s': float(t[-1]), 'inliers': int(keep.sum()), 'points': len(t),
            'residual_px': rms, 'vx_px_s': float(cx[0]), 'vy_px_s': float(cy[1]),
            'image_angle_deg': math.degrees(math.atan2(cy[1],abs(cx[0]))),
            'speed_m_s': math.hypot(cx[0],cy[1])*scale if scale else None,
            'vertical_accel_m_s2': float(2*cy[0]*scale) if scale else None,
            'meters_per_pixel': scale, 'x_span_px': [float(min(values[keep,1])),float(max(values[keep,1]))]}


def shot_advice(fit, target_x=None, target_y=None, tolerance_px=8,
                side_view=False, starts_at_exit=False, current_rpm=None):
    if not fit.get('usable'):
        return {'status': fit.get('reason','轨迹不可用'), 'suggestions': []}
    notes=['核对每条轨迹是否始终跟着同一个球；匹配分数不是识别准确率。']
    result={'status':'仅二维画面分析', 'suggestions':notes}
    if target_x is None or target_y is None or abs(fit['vx_px_s']) < 1:
        notes.append('设置目标位置后可计算轨迹穿过目标竖线时的高低偏差。')
        return result
    tau=(target_x-fit['cx'][1])/fit['cx'][0]
    predicted_y=-float(np.polyval(fit['cy'],tau))
    error=target_y-predicted_y # image y down; positive means flight is above the target
    span=fit['x_span_px']
    extrapolated=not span[0] <= target_x <= span[1]
    result.update(target_time_s=tau+fit['t0'], target_error_px=error,
                  target_extrapolated=extrapolated)
    if tau < 0 or tau > fit['duration_s']+.15:
        notes.append('目标超出可靠时间区间，不给出高低调整结论。')
        return result
    if fit['residual_px'] > max(5,tolerance_px):
        notes.append('拟合误差较大，先检查错跟、遮挡、镜头移动或反弹。')
        return result
    notes.append(('拟合外推到目标，需人工核对。' if extrapolated else '目标在可见轨迹横向范围内。'))
    if abs(error)<=tolerance_px:
        notes.append('画面中目标高度附近通过；仅侧面画面不能判定是否左右偏出或真正进球。')
    else:
        notes.append('画面轨迹偏'+('高' if error>0 else '低')+'；先固定俯仰角小幅试调转速，重复拍摄比较。单次高低偏差不能同时确定角度与转速两项改量。')
    if not(side_view and starts_at_exit and fit['meters_per_pixel']):
        notes.append('量化出射角和速度调整需要：近似垂直于球路的侧拍、同平面尺度标定，且跟踪起点为出球口。')
        return result
    accel=fit['vertical_accel_m_s2']
    if accel is None or not -12.8 <= accel <= -6.8:
        notes.append('竖直加速度与重力近似不符，停止物理调参；检查慢动作时间倍率、标定和透视。')
        return result
    scale=fit['meters_per_pixel']; dx=abs(target_x-fit['cx'][1])*scale
    dy=( -target_y-fit['cy'][2])*scale
    theta=math.radians(fit['image_angle_deg'])
    denominator=2*math.cos(theta)**2*(dx*math.tan(theta)-dy)
    if dx<=0 or denominator<=0:
        notes.append('当前角度在简化抛体模型下无法到达该目标。')
        return result
    speed=fit['speed_m_s']
    discriminant=speed**4-9.80665*(9.80665*dx*dx+2*dy*speed*speed)
    if discriminant>=0:
        angles=[math.degrees(math.atan((speed*speed+sign*math.sqrt(discriminant))/(9.80665*dx))) for sign in [-1,1]]
        result['fixed_speed_angle_options_deg']=angles
        closest=min(angles,key=lambda a:abs(a-fit['image_angle_deg']))
        result['model_angle_change_deg']=closest-fit['image_angle_deg']
        notes.append('角度选项是假设保持球速不变的抛体解；与转速方案二选一测试，不同时调整。舵机位置到实际角度需另行标定。')
    else:
        notes.append('当前球速在简化模型下不存在可达角度，先检查标定或提高球速。')
    needed=math.sqrt(9.80665*dx*dx/denominator)
    ratio=needed/fit['speed_m_s']
    result.update(status='侧拍简化模型估算', tracked_start_angle_deg=fit['image_angle_deg'],
                  estimated_required_speed_m_s=needed, estimated_speed_ratio=ratio)
    if current_rpm and current_rpm>0 and .8<=ratio<=1.2:
        result['model_rpm']=float(current_rpm)*ratio
        result['first_trial_rpm']=float(current_rpm)*max(.97,min(1.03,ratio))
        notes.append('RPM 仅按球速与轮速近似成比例估算；先尝试不超过当前转速 3% 的变化，再实测校正。')
    else:
        notes.append('未给出 RPM 或所需变化过大，不生成 RPM 数值。')
    return result


def rpm_diagnostics(rows, shot_times=(), tolerance_rpm=100):
    if len(rows)<12:
        raise ValueError('转速日志至少需要 12 条记录。')
    t=np.asarray([r['time_s'] for r in rows],float)
    target=np.asarray([r['target_rpm'] for r in rows],float)
    actual=np.asarray([r['actual_rpm'] for r in rows],float)
    if not np.isfinite([t,target,actual]).all() or np.any(np.diff(t)<=0):
        raise ValueError('日志必须为有限数值，time_s 严格递增。')
    if np.any(target<=0) or np.any(actual<0):
        raise ValueError('只导入正转速的启转/运行片段，反馈使用转速绝对值。')
    if np.max(target)-np.min(target)>max(20,np.median(target)*.01):
        raise ValueError('一次仅分析一个恒定目标转速片段，请按目标档位切分日志。')
    interval=float(np.median(np.diff(t)))
    tail=max(4,len(t)//5);err=target-actual
    overshoot=float(max(0,np.max(actual-target)))
    bias=float(np.median(err[-tail:]));ripple=float(np.percentile(actual[-tail:],95)-np.percentile(actual[-tail:],5))
    suggestions=[]
    if interval>.05:suggestions.append('采样较稀，建议至少 20 Hz 记录以观察恢复过程。')
    if abs(bias)>tolerance_rpm:
        suggestions.append('末段存在偏差。先核对输出是否饱和、电压和编码器比例，再单独试调 F；排除这些因素后再考虑少量 I。')
    if overshoot>2*tolerance_rpm or ripple>2*tolerance_rpm:
        suggestions.append('存在超调或末段波动；先确认是否由送球负载导致，再分别小步降低 P 或 I 做对照。不能仅凭波动确定是哪一个增益。')
    if not suggestions:suggestions.append('本段未见明显稳态偏差或大波动；优先比较连续送球后的最低转速和恢复时间。')
    events=[]
    for shot in shot_times:
        if not t[0] <= shot <= t[-1]:
            events.append({'shot_time_s':shot,'status':'射击时间不在日志区间'});continue
        later=[s for s in shot_times if s>shot]
        end=min(shot+1.5,min(later) if later else t[-1])
        indices=np.where((t>=shot)&(t<=end))[0]
        if not len(indices):continue
        minimum=int(indices[np.argmin(actual[indices])]); recovered=None
        # Require a continuous 0.2 s in tolerance, not one crossing.
        for i in indices[indices>=minimum]:
            j=int(np.searchsorted(t,t[i]+.2))
            if j<len(t) and t[j]<=end and np.all(np.abs(err[i:j+1])<=tolerance_rpm):
                recovered=float(t[i]-shot);break
        events.append({'shot_time_s':float(shot),'minimum_rpm':float(actual[minimum]),
                       'drop_rpm':float(target[minimum]-actual[minimum]),
                       'recovery_s':recovered,'window_end_s':float(end)})
    return {'median_sample_interval_s':interval,'tail_bias_rpm':bias,'overshoot_rpm':overshoot,
            'tail_ripple_p90_rpm':ripple,'shots':events,'suggestions':suggestions,
            'limitation':'诊断是调参试验方向，无法从单段运行数据唯一求出 PIDF。需记录当前增益、输出、电压并做受控阶跃测试。'}
