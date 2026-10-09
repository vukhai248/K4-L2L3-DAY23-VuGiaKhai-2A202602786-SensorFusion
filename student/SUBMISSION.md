# Báo cáo bài nộp — Day 23 Sensor Fusion Lab

> Điền file này rồi commit. Cách nộp: [hướng dẫn nộp](../SUBMISSION.md).

## Thông tin học viên

- Họ tên: Vũ Gia Khải
- MSSV: 2A202602786
- Email: 26ai.khaivg@vinuni.edu.vn
- Link repo (fork): https://github.com/vukhai248/K4-L2L3-DAY23-VuGiaKhai-2A202602786-SensorFusion
- Commit hash nộp (`git rev-parse HEAD`): 2464c06283ee91f2a41d91aa67fa6380ea77f44a

## Tóm tắt kết quả

- `fusion_mode` (bắt buộc `compare`), `frames`, `segment`, `seed`: compare, [0, 198], training_segment-1005081002024129653_5313_150_5333_150_with_camera_labels.tfrecord, 0
- `detection.precision`, `detection.recall`, `detection.tp/fp/fn`: 0.9701 (97.01%), 0.7004 (70.04%), 519 / 16 / 222
- `tracking.lidar.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`: 0.1503 m, 502, 11.3436 m^2, 0, 239, 2.5226
- `tracking.fused.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`: 0.1359 m, 502, 9.2668 m^2, 0, 239, 2.5226
- Giải thích khác biệt hai mode, đọc RMSE cùng số ghép và ghost/miss:
  Ở cả hai chế độ, số cặp ghép (matches = 502), số ghost track (ghost_track_frames = 0), số missed ground truth (missed_gt_frames = 239) và số track trung bình (mean_confirmed_tracks = 2.52) là hoàn toàn đồng nhất. Điều này phản ánh chính xác thiết kế track-then-fuse: vòng đời track (khởi tạo, tích lũy score, xác nhận, xóa) chỉ do lượt LiDAR quyết định. Sự khác biệt cốt lõi nằm ở độ chính xác ước lượng vị trí 3D: chế độ Fused đạt RMSE là 0.1359 m, giảm 0.0144 m (cải thiện xấp xỉ 9.6%) so với chế độ LiDAR-only (0.1503 m), với tổng bình phương sai số vị trí sum_sq_err giảm từ 11.3436 m^2 xuống 9.2668 m^2. Việc cập nhật bổ sung quan sát 2D từ camera FRONT giúp tinh chỉnh vector trạng thái vị trí (nhất là theo các trục chiếu vuông góc với tia nhìn) mà không làm phát sinh thêm bất kỳ ghost track nào.

Chạy từ root repo:

```bash
fusion-run-lab --config student/config/paths.yaml --fusion compare --seed 0
```

`rmse = sqrt(sum_sq_err/matches)` trên vị trí 3D của confirmed tracks ghép
một-một với GT xe trong cửa sổ BEV, gate XY **2.0 m**; `null` nếu không có cặp.
Camera dùng tâm hộp 2D ground-truth FRONT có nhiễu seeded, **không** dùng camera
detector. Kết quả này không đo hiệu quả một perception system độc lập với GT.

`grade_run.log` là JSONL, mỗi `(mode,frame)` đúng một record với các trường:
`mode`, `frame`, `det_tp`, `det_fp`, `det_fn`, `valid_gt`, `confirmed`, `matches`,
`sum_sq_err`, `ghosts`, `misses`. Đảm bảo `matches+ghosts==confirmed` và
`matches+misses==valid_gt`; tổng/trung bình record phải khớp `metrics.json`.
File per-mode `metrics_lidar.json`, `metrics_fused.json`, `grade_run_lidar.log`,
`grade_run_fused.log` được giữ để đối chiếu.

## Giải thích ngắn (Parts E–H — tự viết)

1. Khác biệt đo lidar 3D và camera 2D trong EKF (`z`, `R`)?
   - Đo Lidar: Đo trực tiếp vị trí không gian 3D z = [x, y, z]^T (chiều đo m = 3) trong hệ trục cảm biến. Ma trận hiệp phương sai sai số đo R có kích thước 3x3 với các phần tử đường chéo tính bằng mét bình phương (sigma_lidar_x^2, sigma_lidar_y^2, sigma_lidar_z^2 = 0.1^2 m^2). Mô hình đo là tuyến tính, ma trận quan sát H có kích thước 3x6.
   - Đo Camera: Đo vị trí điểm ảnh 2D z = [u, v]^T (chiều đo m = 2) trên mặt phẳng ảnh. Ma trận R có kích thước 2x2 tính bằng pixel bình phương (sigma_cam_i^2, sigma_cam_j^2 = 5.0^2 px^2). Mô hình đo phi tuyến (chiếu pinhole h(x) phụ thuộc vào phép chia độ sâu x_s), đòi hỏi tính ma trận Jacobian H = dh/dx kích thước 2x6 qua quy tắc chuỗi (chain rule).

2. Vì sao cần gating Mahalanobis trước khi gán?
   - Khoảng cách Mahalanobis d^2 = gamma^T * S^(-1) * gamma đo độ lệch giữa phép đo và dự báo có chuẩn hóa theo ma trận hiệp phương sai innovation S = H*P*H^T + R. Khác với khoảng cách Euclidean, khoảng cách Mahalanobis co giãn theo ellipse độ bất định của track: cho phép dung sai lớn hơn theo hướng có độ bất định cao và siết chặt theo hướng có độ chính xác cao.
   - Việc áp dụng cổng chi-square (chi2_gate) trước khi gán giúp loại bỏ sớm các phép đo ngoại lai (clutter/false alarms) hoặc thuộc về đối tượng khác nằm ngoài phân bố xác suất của track, ngăn ngừa gán nhầm dẫn đến phân kỳ bộ lọc EKF.

3. Pipeline là track-then-fuse hay fuse-then-track? Chỉ ra trên log `fusion-run-lab`.
   - Pipeline của lab là track-then-fuse. Duy nhất một tracker quản lý danh tính các đối tượng; mỗi frame EKF predict một lần cho toàn bộ track, sau đó lần lượt cập nhật bằng đo Lidar (AssocL) rồi cập nhật bằng đo camera (AssocC).
   - Trong log grade_run.log, không có bước dung hợp cụm điểm hay ảnh ở mức raw detection trước khi tracking (không phải fuse-then-track). Thay vào đó, log cho thấy cùng một tập hợp confirmed tracks được duy trì liên tục qua các frame và số matches (502), ghosts (0) giữa mode lidar và fused là hoàn toàn trùng khớp từng frame.

4. Nếu camera lệch calibration, triệu chứng gì trên innovation/residual?
   - Nếu camera bị lệch ngoại tại (extrinsic transform) hoặc nội tại (intrinsics), phép chiếu pinhole h(x) sẽ tính ra tọa độ điểm ảnh bị lệch có hệ thống (systematic bias).
   - Triệu chứng: Residual innovation gamma = z - h(x) sẽ không còn có kỳ vọng bằng 0 mà xuất hiện độ lệch trung bình khác 0 rõ rệt. Khoảng cách Mahalanobis d^2 tăng mạnh khiến nhiều phép đo camera hợp lệ bị cổng chi-square loại bỏ (gating rejection). Nếu vẫn lọt qua gate, Kalman gain sẽ kéo vị trí track lệch theo hướng sai số calibration, làm tăng RMSE.

5. Vì sao `associate_and_update(..., sensor)` cần sensor tường minh ở frame rỗng?
   Giải thích vì sao lidar quyết định score/init/delete còn camera chỉ EKF update.
   - Khi frame không có phép đo nào (meas_list rỗng), tracker vẫn bắt buộc phải chạy manage_tracks để xử lý các track không được gán đo (unassigned tracks). Nếu sensor là Lidar, những track nằm trong FOV mà không có đo sẽ bị tính là miss và bị trừ 1/window điểm tồn tại. Cần sensor tường minh để phân biệt lượt Lidar (cần trừ score) và lượt Camera (không được trừ score).
   - Lidar cho vị trí 3D trực quan và bao quát, độ tin cậy hình học cao nên chịu trách nhiệm toàn bộ vòng đời track (khởi tạo track mới, xác nhận khi đủ hit, xóa khi cạn score/bất định lớn). Camera chỉ là phép đo 2D phụ trợ ở góc nhìn hẹp phía trước, không đo trực tiếp khoảng cách 3D độc lập, do đó chỉ được dùng để tinh chỉnh trạng thái EKF mà không được phép tạo, đổi điểm hay xóa track (tránh tạo ghost hoặc xóa nhầm khi xe ra khỏi tầm nhìn hẹp của camera).

6. Nêu điều kiện xác nhận, giữ confirmed sau miss, và điều kiện xóa track.
   - Xác nhận (Confirmation): Mỗi lần có Lidar hit, điểm score tăng thêm +1/window (tối đa 1.0). Khi score > confirmed_threshold (0.8), track chuyển sang trạng thái "confirmed".
   - Giữ confirmed sau miss: Khi track đã ở trạng thái "confirmed", nếu gặp Lidar miss trong FOV (bị trừ -1/window điểm), trạng thái của track vẫn được bảo toàn là "confirmed", không bị hạ cấp về "tentative".
   - Xóa track (Deletion): Track bị xóa nếu thỏa mãn bất kỳ điều kiện nào (OR):
     1. Phương sai vị trí ngang vượt ngưỡng: P[0,0] > max_P hoặc P[1,1] > max_P (max_P = 9.0 m^2).
     2. Track đã confirmed nhưng điểm score < delete_threshold (0.6).
     3. Track chưa confirmed nhưng điểm score <= 0.0.

## Bonus (không bắt buộc)

Liệt kê phần bonus đã làm, file bằng chứng trong `student/bonus/` và kết quả chính
(xem [RUBRIC.md](../RUBRIC.md) mục 2). Không làm thì ghi "Không".

- Không

## Khai báo sử dụng AI (bắt buộc)

Ghi rõ, kể cả khi không dùng ("Không dùng AI"). Xem [RULES.md](../RULES.md) mục 2.

- Công cụ đã dùng (ChatGPT, Copilot, Claude, …): Antigravity AI Assistant
- Dùng cho phần nào (hàm, câu hỏi, debug): Hỗ trợ phân tích codebase, hoàn thiện các hàm EKF 6D trong kalman.py, mô hình đo camera và FOV trong camera_fusion.py, thuật toán gán Mahalanobis chi-square trong association.py, quản lý vòng đời track trong track_management.py, sửa lỗi ép kiểu scalar trên NumPy 2.x, chạy pipeline đánh giá Waymo full compare và soạn thảo giải thích kỹ thuật.
- Cách bạn đã kiểm tra lại (pytest, chạy Waymo, đối chiếu công thức): Đã chạy toàn bộ 128 unit tests với pytest đạt 100% passed; chạy đánh giá thực tế full 198 frames trên Waymo Open Dataset đạt RMSE Lidar 0.1503 m và Fused 0.1359 m (vượt chuẩn <= 0.45 m); và kiểm tra đạt 100% tiêu chí bằng công cụ check_submission.py.

## Checklist nộp

- [x] **Part E–H** trong `workspace/` đã implement; `pytest student/tests -q` không còn `failed`/`xfailed`
- [x] Part A–D: không bắt buộc sửa (hoặc ghi chú nếu bạn đã sửa)
- [x] Lần chạy chấm điểm: `--fusion compare --seed 0`, `frame_start: 0`, `frame_end: 198`
- [x] Đã commit `student/artifacts/metrics*.json` và `student/artifacts/grade_run*.log` (không sửa tay)
- [x] Đã điền đủ file này, gồm khai báo AI
- [x] Không commit dữ liệu Waymo, weights, `paths.yaml`, API key
- [x] `python tools/check_submission.py` báo `KẾT QUẢ: SẴN SÀNG NỘP`
- [x] Đã push và nộp link repo + commit hash trên LMS ([hướng dẫn nộp](../SUBMISSION.md))
