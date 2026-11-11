"""Run with: streamlit run app.py"""
import hashlib
import io
import json
from pathlib import Path
import tempfile
import cv2
import numpy as np
import pandas as pd
import streamlit as st
from analysis import COLORS, read_frame, track, fit_trajectory, shot_advice, rpm_diagnostics

st.set_page_config(page_title='Prototype 球路分析', page_icon='🎯', layout='wide')
st.title('Prototype 四球球路分析')
st.caption('手机视频离线分析 · 按发射顺序编号 · 轨迹可手动修正 · 飞轮转速日志诊断')
video_tab, rpm_tab = st.tabs(['四球轨迹与射击建议', '飞轮 PIDF 诊断'])

def image_rgb(frame):
    return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)


def save_upload(upload):
    digest=hashlib.sha256(upload.getbuffer()).hexdigest()[:20]
    root=Path(tempfile.gettempdir())/'prototype-shot-analyzer'
    root.mkdir(exist_ok=True)
    path=root/(digest+Path(upload.name).suffix.lower())
    if not path.exists():path.write_bytes(upload.getbuffer())
    return path,digest


def json_ready(value):
    if isinstance(value,dict):return {k:json_ready(v) for k,v in value.items()}
    if isinstance(value,(tuple,list)):return [json_ready(x) for x in value]
    if isinstance(value,np.generic):return value.item()
    return value


with video_tab:
    st.write('先为每个球选一段从出球到接近目标的飞行区间，并在起始帧标出球中心。程序在各自区间跟踪，避免把上一球的反弹当成下一球。')
    upload=st.file_uploader('导入手机原视频',type=['mp4','mov','avi','m4v'])
    if upload:
        path,digest=save_upload(upload)
        if st.session_state.get('video_digest')!=digest:
            st.session_state.video_digest=digest
            st.session_state.pop('tracks',None)
            st.session_state.pop('configs',None)
        cap=cv2.VideoCapture(str(path))
        fps=cap.get(cv2.CAP_PROP_FPS); frames=cap.get(cv2.CAP_PROP_FRAME_COUNT)
        w=int(cap.get(cv2.CAP_PROP_FRAME_WIDTH));h=int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT));cap.release()
        if fps<=0 or frames<=1 or w<=0 or h<=0:
            st.error('视频无法解码。请导出 H.264 MP4 后重试。')
        else:
            duration=frames/fps
            st.video(str(path))
            st.caption(f'{w} × {h} · 文件帧率 {fps:.2f} fps · 文件时长 {duration:.2f} s。慢动作导出视频需填写下面的时间倍率。')
            st.subheader('1 选择四次发射与球的位置')
            st.info('先在播放器记下每次出球的大致时刻。展开下面每个球，在起始帧找到中心坐标；X 从左到右，Y 从上到下。选框需包含球边缘。时间窗口可重叠，但每个起点必须指向不同一次发射。')
            configs=[]
            for ball in range(4):
                with st.expander(f'球 {ball+1} 起点与跟踪区间',expanded=ball==0):
                    a,b=st.columns(2)
                    default=min(duration-.05, ball*duration/4)
                    start=a.number_input('起始时刻 s',min_value=0.,max_value=max(0.,duration-1/fps),value=max(0.,default),step=1/fps,key=f'{digest}_start{ball}',format='%.3f')
                    end=b.number_input('结束时刻 s',min_value=0.,max_value=float(duration),value=float(min(duration,default+min(1.2,duration/4))),step=.05,key=f'{digest}_end{ball}',format='%.3f')
                    try:
                        frame=read_frame(path,start)
                        x=st.slider('球中心 X 像素',0,w-1,w//3,key=f'{digest}_x{ball}')
                        y=st.slider('球中心 Y 像素',0,h-1,h//2,key=f'{digest}_y{ball}')
                        r=st.slider('球半径与边缘范围 px',3,max(4,min(w,h)//5),max(3,min(15,min(w,h)//10)),key=f'{digest}_r{ball}')
                        preview=frame.copy();cv2.rectangle(preview,(x-r,y-r),(x+r,y+r),COLORS[ball],2)
                        cv2.drawMarker(preview,(x,y),COLORS[ball],cv2.MARKER_CROSS,20,2)
                        cv2.putText(preview,f'Ball {ball+1} ({x},{y})',(12,32),cv2.FONT_HERSHEY_SIMPLEX,.8,COLORS[ball],2)
                        st.image(image_rgb(preview),caption='拖动坐标，使十字中心对准刚飞出的球。')
                        configs.append(dict(start=start,end=end,seed_x=x,seed_y=y,radius=r))
                    except ValueError as error:st.error(str(error))
            with st.expander('跟踪参数'):
                search=st.slider('每帧搜索半径 px',20,400,100,10)
                threshold=st.slider('最低外观匹配分数',.2,.95,.55,.05)
            if st.button('跟踪四个球',type='primary',disabled=len(configs)!=4):
                if any(configs[i]['start']>=configs[i+1]['start'] for i in range(3)):
                    st.error('请按实际发射先后设置严格递增的四个起始时间。')
                else:
                    tracks=[]
                    with st.spinner('逐帧跟踪…'):
                        for i,cfg in enumerate(configs):
                            try:
                                pts,_=track(path,**cfg,search=search,threshold=threshold)
                                tracks.append(pts)
                                st.write(f'球 {i+1}：{len(pts)} 个有效位置')
                            except ValueError as error:
                                st.error(f'球 {i+1}：{error}');tracks.append([])
                    st.session_state.track_revision=st.session_state.get('track_revision',0)+1
                    st.session_state.tracks=tracks
                    st.session_state.configs=[dict(c) for c in configs]
                    st.session_state.tracking_params=(search,threshold)
            tracks=st.session_state.get('tracks')
            if tracks is not None:
                if configs!=st.session_state.get('configs') or (search,threshold)!=st.session_state.get('tracking_params'):
                    st.warning('起点、窗口或跟踪参数已改变；当前显示上次结果，点击“跟踪四个球”重新计算。')
                st.subheader('2 核对轨迹与修正漏检')
                rows=[]
                for i,pts in enumerate(tracks):
                    for p in pts:rows.append(dict(ball=i+1,**p))
                original=pd.DataFrame(rows,columns=['ball','frame','t','x','y','score'])
                st.caption('编辑或添加坐标、删除错点。新增点需填写视频文件中的 t 秒；frame 和 score 可留空。每球内部时间不能重复。')
                edits=st.data_editor(original,num_rows='dynamic',key=f'points_{digest}_{st.session_state.get("track_revision",0)}',hide_index=True)
                corrected=[[] for _ in range(4)]
                valid=True
                try:
                    for _,row in edits.iterrows():
                        ball=float(row['ball']);t=float(row['t']);x=float(row['x']);y=float(row['y'])
                        if not all(np.isfinite([ball,t,x,y])) or int(ball)!=ball or not 1<=ball<=4 or not 0<=t<=duration or not 0<=x<w or not 0<=y<h:
                            raise ValueError('ball 须为 1 至 4；t、x、y 须在视频范围内。')
                        corrected[int(ball)-1].append(dict(t=t,x=x,y=y))
                    for pts in corrected:pts.sort(key=lambda p:p['t'])
                except (ValueError,TypeError) as error:
                    st.error(f'坐标表无效：{error}');valid=False
                if valid:
                    try:
                        base=read_frame(path,st.session_state.configs[0]['start'])
                        for i,pts in enumerate(corrected):
                            if not pts:continue
                            xy=np.array([[int(p['x']),int(p['y'])] for p in pts],np.int32)
                            cv2.polylines(base,[xy],False,COLORS[i],3)
                            for coord in xy[::max(1,len(xy)//25)]:cv2.circle(base,tuple(coord),3,COLORS[i],-1)
                            cv2.putText(base,str(i+1),tuple(xy[0]),cv2.FONT_HERSHEY_SIMPLEX,1,COLORS[i],3)
                        st.image(image_rgb(base),caption='四球轨迹叠加。曲线仅连接有效观测，跨越遮挡处的连线不代表已识别到球。')
                        ok,png=cv2.imencode('.png',base)
                        if ok:st.download_button('下载轨迹叠加图',png.tobytes(),'four-ball-tracks.png','image/png')
                    except ValueError as error:st.error(str(error))
                    st.subheader('3 标定目标与分析')
                    side=st.checkbox('固定侧拍，镜头近似垂直于球的飞行平面，相机没有移动')
                    exit_start=st.checkbox('四段轨迹起点均为球刚离开发射器的位置')
                    scale_on=st.checkbox('提供同一飞行平面的已知长度标定')
                    scale=None
                    if scale_on:
                        a,b=st.columns(2)
                        length=a.number_input('已知长度 m',min_value=.001,value=1.,step=.1)
                        pixels=b.number_input('该长度在画面中的像素数',min_value=1.,value=200.,step=1.)
                        if side:scale=length/pixels
                        else:st.warning('当前没有确认侧拍几何条件，仅输出画面坐标。')
                    a,b,c=st.columns(3)
                    target_x=a.number_input('目标入口中心 X px',min_value=0.,max_value=float(w-1),value=float(w*.8))
                    target_y=b.number_input('目标入口中心 Y px',min_value=0.,max_value=float(h-1),value=float(h*.4))
                    tolerance=c.number_input('画面高低容差 px',min_value=1.,value=8.)
                    with st.expander('核对目标位置'):
                        target_preview=base.copy()
                        cv2.drawMarker(target_preview,(int(target_x),int(target_y)),(255,255,255),cv2.MARKER_CROSS,32,3)
                        cv2.line(target_preview,(int(target_x),0),(int(target_x),h-1),(200,200,200),1)
                        st.image(image_rgb(target_preview),caption='白色十字应位于目标入口中心；竖线是轨迹比较的位置。')
                    a,b=st.columns(2)
                    rpm=a.number_input('这组四球的目标飞轮 RPM，未知填 0',min_value=0.,value=0.,step=100.)
                    time_factor=b.number_input('文件秒 / 实际秒，普通视频填 1，8 倍慢动作填 8',min_value=.01,max_value=100.,value=1.,step=1.)
                    summaries=[];reports=[]
                    for i,pts in enumerate(corrected):
                        real=[dict(p,t=p['t']/time_factor) for p in pts]
                        fit=fit_trajectory(real,scale)
                        advice=shot_advice(fit,target_x,target_y,tolerance,side,exit_start,rpm)
                        reports.append(dict(ball=i+1,fit=fit,advice=advice))
                        summaries.append({'球':i+1,'有效点':len(pts),'拟合误差 px':fit.get('residual_px'),
                                          '目标处偏高 px':advice.get('target_error_px'),
                                          '起点球速 m/s':fit.get('speed_m_s'),
                                          '起点投影角 °':fit.get('image_angle_deg') if side else None,
                                          '模型所需球速 m/s':advice.get('estimated_required_speed_m_s')})
                        with st.expander(f'球 {i+1} 分析：{advice["status"]}'):
                            st.write(advice)
                    st.dataframe(pd.DataFrame(summaries),hide_index=True)
                    st.caption('高低偏差是球通过目标竖线时的二维拟合差，正值为偏高。速度只在侧拍标定后显示；起点不在出口时，它是跟踪起点速度。所有物理估算均忽略空气阻力和旋转升力。')
                    st.download_button('导出分析 JSON',json.dumps(json_ready({'shots':reports,'time_factor':time_factor}),ensure_ascii=False,indent=2),'shot-report.json','application/json')
                    st.download_button('导出修正后的轨迹 CSV',edits.to_csv(index=False).encode('utf-8-sig'),'tracks.csv','text/csv')
                    st.info('如果后几球逐渐偏低，请用下一个标签页核对飞轮掉速。四次球路本身不能证明是 PID 问题，还可能与送球间隔、球体差异或机械打滑有关。')
    else:
        st.info('尚未导入视频。固定手机机位，完整拍到出球口和目标，尽量使用 120/240 fps 原始拍摄；不要手持跟拍。')

with rpm_tab:
    st.subheader('用转速反馈分析连续发射')
    st.write('导入 CSV：time_s,target_rpm,actual_rpm。可另外记录 output、battery_v、kP、kI、kD、kF；当前工具只对转速作定量诊断，不自动修改机器人参数。')
    st.download_button('下载日志格式样例','time_s,target_rpm,actual_rpm,output,battery_v\n0.00,5000,0,1.0,12.6\n0.02,5000,300,1.0,12.6\n','rpm-template.csv','text/csv')
    logfile=st.file_uploader('导入飞轮日志',type=['csv'],key='rpm_upload')
    event_text=st.text_input('四次射击时间，使用日志时间轴，英文逗号分隔','')
    tol=st.number_input('转速恢复容差 RPM',min_value=1.,value=100.)
    st.caption('视频与日志不自动同步；先用可见的第一次出球或同步标记对齐时间，再填写日志中的射击时刻。')
    if logfile:
        try:
            df=pd.read_csv(io.BytesIO(logfile.getvalue()))
            required={'time_s','target_rpm','actual_rpm'}
            if not required<=set(df.columns):raise ValueError('缺少必需列：'+', '.join(sorted(required-set(df.columns))))
            times=sorted(float(s.strip()) for s in event_text.split(',') if s.strip())
            if not np.isfinite(times).all():raise ValueError('射击时间必须为有限数值。')
            result=rpm_diagnostics(df.to_dict('records'),times,tol)
            st.line_chart(df.set_index('time_s')[['target_rpm','actual_rpm']])
            st.json(result)
            st.download_button('导出转速诊断',json.dumps(result,ensure_ascii=False,indent=2),'rpm-report.json','application/json')
        except (ValueError,TypeError,KeyError,pd.errors.ParserError) as error:st.error(str(error))
