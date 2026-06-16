function runFOPIDmodel1()

    clc;
    close all;
    clearvars;
    warning off;
    
    tic_total = tic;  % تایمر کل برنامه
    
    
    %% 1) بارگذاری نقشه و ساخت occupancy map
    load Obstaculo2.mat;  % متغیر environment = Obstaculo2
    environment = Obstaculo2 ; 
    refMap       = binaryOccupancyMap(environment, 1);   % همان سطر قبلی شما

   
planningMap = copy(refMap);           % ⇦ نسخهٔ مخصوص مسیر‌یابی
robotRadius = 0.3;                    % شعاع واقعی ربات (متر)
runtimeMap  = copy(refMap);   % نقشه‌ای که حسگرها می‌بینند
inflate(planningMap, robotRadius);   % فقط planningMap باد شود
refMapDyn    = runtimeMap;
% (اختیاری) اگر می‌خواهید پس‌زمینه بدون تورم نمایش داده شود
refMapDisp = copy(refMap);            % فقط برای نمایش ثابت

% --- سقف و کف سرعت و آستانه‌های فاصله برای زمان‌بندی سرعت ---
v_max   = 1;   % [m/s] حداکثر سرعت وقتی از هدف دوریم (توان/ولتاژ بیشتر)
v_min   = 0.2;   % [m/s] حداقل سرعت نزدیک هدف (حرکت نرم)
R_fast  = 8.0;    % [m] اگر فاصله > R_fast → با v_max برو
R_slow  = 0.5;    % [m] اگر فاصله < R_slow → با v_min برو


%% === FOPID setup for Tracking (نسخهٔ پایدار) ===
% ---------- ضرایب ----------
Kp_psi = 10;   Ki_psi =0.01;   Kd_psi =0.2;
Kp_u = 10;      % قبلاً 10
Ki_u = 0.005;     % قبلاً 0.01
Kd_u = 0.15;      % قبلاً 0.2


lambda_psi = 0.3;   mu_psi = 0.3;
lambda_u   = 0.3;   mu_u   = 0.3;

wL = 0.7;        wH = 6;     % باند فرکانسی
N  = 2;                     % مرتبهٔ تقریب
Ts = 0.05;                  % = stepSize_time

s = fotf('s');

% ===== Heading ψ =====
Cint = oustapp(1/s^lambda_psi , wL, wH, N);
Cder = oustapp(  s^mu_psi     , wL, wH, N);
Cpsi = Kp_psi + Ki_psi*Cint + Kd_psi*Cder;
CdPsi = c2d(tf(Cpsi), Ts, 'tustin');
[b_psi,a_psi] = tfdata(CdPsi,'v');
z_int_psi = zeros(max(numel(a_psi),numel(b_psi))-1 ,1);

% ===== Speed u =====
Cint = oustapp(1/s^lambda_u , wL, wH, N);
Cder = oustapp(  s^mu_u     , wL, wH, N);
Cu   = Kp_u + Ki_u*Cint + Kd_u*Cder;
CdU  = c2d(tf(Cu), Ts, 'tustin');
[b_u,a_u] = tfdata(CdU,'v');
z_int_u   = zeros(max(numel(a_u),numel(b_u))-1 ,1);

assert(isstable(CdPsi) && isstable(CdU), 'FOPID ناپایدار شد!')


controlMode    = 1;
% 3) تنظیم data
data.refMap     = planningMap;   % الگوریتم مسیر‌یابی
data.refMapDisp = planningMap;   % نمایش ثابت (می‌خواهید شکل واقعی نشان داده شود)

    mapXLimits = refMap.XWorldLimits;  
    mapYLimits = refMap.YWorldLimits;  
    fprintf('Map X limits: [%.1f %.1f], Y limits: [%.1f %.1f]\n',...
        mapXLimits(1), mapXLimits(2), mapYLimits(1), mapYLimits(2));
numObs      = 9;            % ← حالا 4 تا مانع داریم
xFixed      = 10;           % برای 3 مانع عمودی
startYlist  = [3 6 2];     
vYlist      = [0.5 0.6 0.5];
radObs      = 0.4;

for i = 1:3
    movObs(i).x  = xFixed;
    movObs(i).y0 = startYlist(i);
    movObs(i).v  = vYlist(i);
    movObs(i).r  = radObs;
    
    prevCells{i} = [];
    movObs(i).plotArgs = {'ko','MarkerFaceColor','k','MarkerSize',radObs*30};
end

% — مانع چهارم: حرکت در راستای x با y ثابت —
k = 4;
movObs(k).x0       = 2;     
movObs(k).y0       = 8;     
movObs(k).v        = 0.8;   
movObs(k).r        = radObs;
prevCells{k}       = [];
movObs(k).plotArgs = {'ro','MarkerFaceColor','r','MarkerSize',radObs*30};
% — مانع پنجم: حرکت عمودی روی x=12، نمایش مثلث رو به بالا —
k = 5;
movObs(k).x   = 12;       % x ثابت
movObs(k).y0  = 2;        % نقطه شروع y
movObs(k).v   = 0.4;      % سرعت
movObs(k).r   = radObs;
prevCells{k}  = [];
% مثلث رو به بالا با رنگ سبز
movObs(k).plotArgs = {'^','MarkerFaceColor','g','MarkerSize',radObs*30};

% — مانع ششم: حرکت عمودی روی x=18، نمایش مثلث رو به پایین —
k = 6;
movObs(k).x   = 18;
movObs(k).y0  = 18;
movObs(k).v   = 0.3;
movObs(k).r   = radObs;
prevCells{k}  = [];
% مثلث رو به پایین با رنگ آبی
movObs(k).plotArgs = {'v','MarkerFaceColor','b','MarkerSize',radObs*30};

% — مانع هفتم: حرکت افقی روی y=5، نمایش ستاره سبز —
k = 7;
movObs(k).x0      = 1;      % مختصات اولیه x
movObs(k).y0      = 12;      % y ثابت
movObs(k).v       = 0.7;    % سرعت
movObs(k).r       = radObs;
prevCells{k}      = [];
movObs(k).plotArgs = {'*','MarkerFaceColor','g','MarkerSize',radObs*30};

% — مانع هشتم: حرکت افقی روی y=15، نمایش ستاره آبی —
k = 8;
movObs(k).x0      = 3;
movObs(k).y0      = 15;
movObs(k).v       = 0.6;
movObs(k).r       = radObs;
prevCells{k}      = [];
movObs(k).plotArgs = {'d','MarkerFaceColor','b','MarkerSize',radObs*30};
% — مانع نهم: حرکت دایره‌ای با R=2 حول نقطه (15,10) —
k = 9;
movObs(k).cx = 15;      % مرکز گردش
movObs(k).cy = 10;
movObs(k).R  = 2;       % شعاع مسیر
movObs(k).w  = 0.4;     % سرعت زاویه‌ای [rad/s]
movObs(k).r  = radObs;  % شعاع جسم
prevCells{k} = [];

% لوزی زرد برای نمایش
movObs(k).plotArgs = {'p','MarkerFaceColor','y','MarkerEdgeColor','k','MarkerSize',radObs*30};


    %% 2) تعریف نقاط شروع و هدف
    startPoint = [5, 5];
    finalGoal  = [20, 20];  
    
    %% 3) تنظیم ساختار داده برای الگوریتم دراگون‌فلای
    nHandlePoints = 3;  
    n = nHandlePoints;
    data.n = n;
    data.nvar = 2*n;
    
    data.xmin = mapXLimits(1);
    data.xmax = mapXLimits(2);
    data.ymin = mapYLimits(1);
    data.ymax = mapYLimits(2);
    
    data.LB = [data.xmin*ones(1,n), data.ymin*ones(1,n)];
    data.UB = [data.xmax*ones(1,n), data.ymax*ones(1,n)];
    
    data.xs = startPoint(1);
    data.ys = startPoint(2);
    data.xt = finalGoal(1);
    data.yt = finalGoal(2);
    
    data.refMapDisp = refMapDisp;
    data.beta = 1500;  % وزن جریمه برخورد

    % ------ پارامترهای جریمهٔ نزدیکی (Proximity Penalty) ------
    data.clearanceThr =0.4;   % حداقل فاصلهٔ متوسط از موانع [m]
    data.wProx        = 500;   % وزن جریمه‌ی نزدیکی
    % ----------------------------------------------------------

    % ------ ایجاد distanceMap و اضافه کردن به data ------
occMat        = occupancyMatrix(planningMap);   % نه runtimeMap
dm            = bwdist(occMat);
data.distanceMap = flipud(dm);

    %% 4) مسیر‌یابی با الگوریتم دراگون‌فلای
    fprintf('\n>>> Starting path planning with Dragonfly algorithm...\n');
    tic_df = tic;
    [gpop, bestCost, bestCostHistory] = DragonflyPathPlanning_DF(data);

    elapsedTimeDF = toc(tic_df);
    fprintf('Dragonfly path planning execution time: %.2f sec\n', elapsedTimeDF);
    
    if ~gpop.IsFeasible
        fprintf('\nNo feasible path was found.\n');
        return;
    end
    fprintf('Best path cost found = %.3f\n', bestCost);
    
    % بردار مسیر خام (اسپلاین دراگون‌فلای)
    xx = gpop.info.xx;
    yy = gpop.info.yy;
    
    %% (اختیاری) 4-1) مرحلهٔ بهینه‌سازی موضعی با فاصله‌نگاشت
  

 [xx_opt, yy_opt] = localPathOptimization(xx, yy, data.refMap, data.distanceMap);
    
    % بروزکردن اطلاعات مسیر در gpop
    gpop.info.xx = xx_opt;
    gpop.info.yy = yy_opt;
    xx = xx_opt;
    yy = yy_opt;
    
 

        pathLengthOpt = sum( sqrt( diff(xx).^2 + diff(yy).^2 ) );
    fprintf('Optimized Dragonfly Path Length: %.3f meters\n\n', pathLengthOpt);
 % ---- تعیین طول مطلوب هر سگمنت ----------
segmentLen      = 2;            % [m] مقدار دلخواه (2–3 m خوب است)

% ---- تعداد چک‌پوینت پویا ---------------
nDesired = max(4, ceil(pathLengthOpt/segmentLen) + 1);
dx      = diff(xx);
dy      = diff(yy);
d       = sqrt(dx.^2 + dy.^2);

% ← اینجا تغییر بده
cumDist = [0, cumsum(d)];  

totalL  = cumDist(end);

    % 3) نقاط یکنواخت روی طول
    dists = linspace(0, totalL, nDesired);

    % 4) اینترپوله کردن برای گرفتن مختصات
    xxu = interp1(cumDist, xx, dists);
    yyu = interp1(cumDist, yy, dists);

    checkpoints = [xxu', yyu'];
    %% ----------------------------------------------------------
    tic_control = tic;
    
    %% 5) پیاده‌سازی کنترل (مثلاً فازی یا PID + اجتناب از مانع)
    % مقداردهی اولیه
    state_initial = zeros(1,24);
    state_initial(19) = checkpoints(1,1);
    state_initial(20) = checkpoints(1,2);
    
    startPointSim = checkpoints(1,:);
    finalGoalSim = checkpoints(end,:);
    fprintf('Start point (Dragonfly):  [%.1f, %.1f]\n', startPointSim(1), startPointSim(2));
    fprintf('Final goal (Dragonfly):   [%.1f, %.1f]\n', finalGoalSim(1), finalGoalSim(2));
    

    % تنظیم سنسورها
    lidar_left = rangeSensor;
    lidar_left.Range = [0,10];
    lidar_left.HorizontalAngle = [pi/6, pi/2];
    
    lidar_right = rangeSensor;
    lidar_right.Range = [0,10];
    lidar_right.HorizontalAngle = [-pi/2, -pi/6];
    
    lidar_front = rangeSensor;
    lidar_front.Range = [0,10];
    lidar_front.HorizontalAngle = [-pi/9, pi/9];
    
   
    
    tolerance = 0.5;
    counterCheckpoint = 1;
    simulationTime_total = 500; 
    stepSize_time = 0.05;  
    timeSteps_total = simulationTime_total / stepSize_time;
    
    global path_x path_y;
    path_x = [];
    path_y = [];

    voltage_left_values = [];
voltage_right_values = [];
time_voltage = [];  % آرایه ثبت زمان
distanceToCheckpoint_array = [];

    state = state_initial;

    isRobotStopped  = false;
stopDist   = 1.7;   % اگر d < 1.6 → توقف
resumeDist = 1.5;   % اگر d > 1.9 → ادامه (بزرگ‌تر از stopDist)

isAvoidActive   = false;   % <‑‑ اضافه شد

    %% آرایه‌های ثبت خطا (برای محاسبه شاخص‌های خطا)
angle_error_array = [];     % خطای زاویه (به رادیان)
   speed_error_array = [];     % خطای سرعت (m/s)  ← اضافه شد

  
    fisAvoid = a107();


% ----- برای کنترل اجتناب: شمارش firing هر قاعده -----
numRulesAvoid   = numel(fisAvoid.Rules);
ruleCountAvoid  = zeros(numRulesAvoid,1);
ruleEnergyAvoid = zeros(numRulesAvoid,1);

    
    fig = figure('Name','Dragonfly + PIDControl + Obstacle Avoidance');
    show(refMapDisp); hold on;
    plot(startPointSim(1), startPointSim(2), 'gx', 'MarkerSize',8, 'LineWidth',2);
    text(startPointSim(1)+0.3, startPointSim(2)+0.3,...
         'Start','Color','green','FontSize',10,'FontWeight','bold');
    plot(checkpoints(:,1), checkpoints(:,2), 'rx-', 'MarkerSize',5, 'LineWidth',1.5);
    title('Trajectory: PID + Obstacle Avoidance + Dragonfly Path');
    disp('Starting control simulation...');
    % --- تنظیمات اجتناب و فیلتر فاصلهٔ جلو (یک‌بار قبل از حلقه) ---
avoidEnterDist = 1.6;      % ورود به حالت اجتناب
avoidExitDist  = 1.8;      % خروج از حالت اجتناب
isAvoidActive  = false;    % وضعیت اجتناب (state)

alpha          = 0.4;      % ضریب EMA برای هموارسازی فاصله جلو (0<alpha<1)
dfront_filt    = NaN;      % مقدار اولیهٔ فیلتر جلو
%% --- Low-pass filters for angle & speed errors ---
fc_ang = 1.0;                  % Hz  (برای زاویه؛ 0.8 تا 1.5 امتحان کن)
fc_spd = 0.8;                  % Hz  (برای سرعت؛ 0.5 تا 1.2 امتحان کن)

alpha_ang = 1 - exp(-2*pi*fc_ang*stepSize_time);
alpha_spd = 1 - exp(-2*pi*fc_spd*stepSize_time);

eAng_f = 0;                    % حالت فیلتر خطای زاویه
eSpd_f = 0;                    % حالت فیلتر خطای سرعت

dF = inf;   % مقدار اولیه: یعنی «جلوی ربات باز است»
goalDist_array = [];

    %% حلقه اصلی شبیه‌سازی
    for timeStep = 1:timeSteps_total
        
currentHeadingAngle = state(timeStep, 24);          % theta
currentLocation     = [state(timeStep,19), state(timeStep,20)];  % (x,y)
checkpoint = checkpoints(counterCheckpoint,:);
distanceToCheckpoint = norm(checkpoint - currentLocation);

% --- این دو خط را همین‌جا بگذارید (قبل از هر استفاده از goalDist) ---
goalDist = norm(currentLocation - finalGoal);
if timeStep == 1
    goalDist_array = goalDist;
else
    goalDist_array(end+1) = goalDist;
end
% --- زمان‌بندی پیوسته سرعت مرجع بر حسب فاصله تا هدف ---
alpha_g = (goalDist - R_slow) / (R_fast - R_slow);  % نرمال‌سازی
alpha_g = max(0, min(1, alpha_g));                  % محدودسازی به [0,1]
desiredSpeed_now = v_min + (v_max - v_min)*alpha_g; % نزدیک هدف→v_min، دور از هدف→v_max

    % بررسی رسیدن به چک‌پوینت
    if distanceToCheckpoint < tolerance
        counterCheckpoint = counterCheckpoint + 1;
        if counterCheckpoint > size(checkpoints,1)
            disp('All checkpoints (PSO path) have been reached.');
            break;
        else
            checkpoint = checkpoints(counterCheckpoint,:);
        end
    end

    % بررسی رسیدن به هدف نهایی
    finalGoalDistance = norm(finalGoal - currentLocation);
    if finalGoalDistance < tolerance
        disp('Final Target reached. Stopping the robot.');
        % می‌توانید ولتاژها را صفر کنید
        voltage_left=0; 
        voltage_right=0;
        state(timeStep+1,:)= state(timeStep,:);
        break;
    end

    % % در صورت تمایل: خواندن سنسورها
    robotPose = [currentLocation, currentHeadingAngle];
t_curr = (timeStep-1)*stepSize_time;

% — ۳ مانع عمودی —
for k = [1,2,3,5,6]
    movObs(k).y = movObs(k).y0 + movObs(k).v * t_curr;
    [XX,YY] = meshgrid(linspace(movObs(k).x-movObs(k).r, movObs(k).x+movObs(k).r, 3), ...
                       linspace(movObs(k).y-movObs(k).r, movObs(k).y+movObs(k).r, 3));
    cellsNow = [XX(:), YY(:)];
    if ~isempty(prevCells{k})
        setOccupancy(refMapDyn, prevCells{k}, 0);
    end
    setOccupancy(runtimeMap, cellsNow, 1);

    prevCells{k} = cellsNow;
end

% — مانع چهارم: حرکت در x با y ثابت —
k = 4;
movObs(k).x = movObs(k).x0 + movObs(k).v * t_curr;
movObs(k).y = movObs(k).y0;   % ثابت
[XX,YY] = meshgrid(linspace(movObs(k).x-movObs(k).r, movObs(k).x+movObs(k).r, 3), ...
                   linspace(movObs(k).y-movObs(k).r, movObs(k).y+movObs(k).r, 3));
cellsNow = [XX(:), YY(:)];
if ~isempty(prevCells{k})
    setOccupancy(refMapDyn, prevCells{k}, 0);
end
setOccupancy(runtimeMap, cellsNow, 1);

prevCells{k} = cellsNow;
for k = 7:8
    movObs(k).x = movObs(k).x0 + movObs(k).v * t_curr;
    movObs(k).y = movObs(k).y0;
    [XX,YY] = meshgrid( linspace(movObs(k).x-movObs(k).r, movObs(k).x+movObs(k).r,3), ...
                       linspace(movObs(k).y-movObs(k).r, movObs(k).y+movObs(k).r,3) );
    cellsNow = [XX(:), YY(:)];
    if ~isempty(prevCells{k})
        setOccupancy(refMapDyn, prevCells{k}, 0);
    end
    setOccupancy(runtimeMap, cellsNow, 1);

    prevCells{k} = cellsNow;
end

% — مانع نهم: مسیر دایره‌ای —
k = 9;
theta = movObs(k).w * t_curr;                % زاویه فعلی
movObs(k).x = movObs(k).cx + movObs(k).R * cos(theta);
movObs(k).y = movObs(k).cy + movObs(k).R * sin(theta);

[XX,YY] = meshgrid(linspace(movObs(k).x-movObs(k).r, movObs(k).x+movObs(k).r,3), ...
                   linspace(movObs(k).y-movObs(k).r, movObs(k).y+movObs(k).r,3));
cellsNow = [XX(:), YY(:)];
if ~isempty(prevCells{k})
    setOccupancy(refMapDyn, prevCells{k}, 0);
end
setOccupancy(runtimeMap, cellsNow, 1);

prevCells{k} = cellsNow;

% --- خواندن پرتوها ---
[r_f,  ~] = lidar_front(robotPose,  runtimeMap);
[r_fl, ~] = lidar_left(robotPose,   runtimeMap);
[r_fr, ~] = lidar_right(robotPose,  runtimeMap);

% --- کمینهٔ غیر-NaN با fallback = برد سنسور ---
maxR_f  = lidar_front.Range(2);
maxR_fl = lidar_left.Range(2);
maxR_fr = lidar_right.Range(2);


rf  = r_f(~isnan(r_f));   if isempty(rf),  d_front = maxR_f;  else, d_front = min(rf);  end
rfl = r_fl(~isnan(r_fl)); if isempty(rfl), d_fl    = maxR_fl; else, d_fl    = min(rfl); end
rfr = r_fr(~isnan(r_fr)); if isempty(rfr), d_fr    = maxR_fr; else, d_fr    = min(rfr); end

% --- EMA برای فاصلهٔ جلو ---
if isnan(dfront_filt), dfront_filt = d_front; end
dfront_filt = alpha*d_front + (1-alpha)*dfront_filt;
dF = dfront_filt;   % معیار اصلی برای تصمیم اجتناب



    % --- بررسی نزدیک بودن موانع متحرک ---
    minDistMovObs = inf;
    for k = 1:numObs
        obsPos = [movObs(k).x, movObs(k).y];
        d = norm(currentLocation - obsPos) - movObs(k).r;   % فاصله تا سطح مانع
        minDistMovObs = min(minDistMovObs, d);
    end
% بعد از محاسبهٔ minDistMovObs
if timeStep == 1
    obsDist_array = minDistMovObs;
else
    obsDist_array(end+1) = minDistMovObs;
end

if ~isRobotStopped && minDistMovObs < stopDist
    isRobotStopped = true;                 % ← کامنت را بردارید
    disp("Moving obstacle too close → ROBOT STOPPED");

elseif  isRobotStopped && minDistMovObs >= resumeDist
    isRobotStopped = false;                % ← و این خط
    disp("Obstacle cleared → ROBOT RESUMES");
end


    % اگر ربات باید متوقف بماند، ولتاژها را صفر کرده و ادامه حلقه را رد کن
    if isRobotStopped
        voltage_left  = 0;
        voltage_right = 0;


        % به‌روزرسانی وضعیت دینامیکی با ولتاژ صفر
        [dstate, state(timeStep,:)] = DynamicalModel([0;0;0;0], state(timeStep,:), stepSize_time);
        if timeStep < timeSteps_total
            state(timeStep+1,:) = state(timeStep,:) + dstate * stepSize_time;
        end

        % رسم صحنه در حالت توقف

        plot(path_x, path_y, 'k--','LineWidth',1.5);
        for k = 1:numObs
            plot(movObs(k).x, movObs(k).y, movObs(k).plotArgs{:});
        end
        DrawRobot(0.2, state(timeStep,19), state(timeStep,20), state(timeStep,24), 'm');
        title('Robot stopped – moving obstacle too close');
       
      % continue
    end

    % محاسبه خطای زاویه به چک‌پوینت:
    angleToGoal = atan2(checkpoint(2)-currentLocation(2), checkpoint(1)-currentLocation(1));
    
angleError_raw = wrapToPi(angleToGoal - currentHeadingAngle);
eAng_f    = eAng_f + alpha_ang*(angleError_raw - eAng_f);   % LPF
angleError = eAng_f;                                        % خطای زاویه‌ی صاف‌شده



    % چون تابع فازی شما از -400 تا 400 تنظیم شده، به درجه تبدیل می‌کنیم
    turning_angle_deg = rad2deg(angleError);


% --- هیسترزیس روی فاصلهٔ جلو (dF) ---
if ~isAvoidActive && dF < avoidEnterDist
    isAvoidActive = true;
elseif isAvoidActive && dF > avoidExitDist
    isAvoidActive = false;
end




if isAvoidActive
    controlMode = 2;

   
  inAvoid = [turning_angle_deg, d_fl, dF, d_fr];   % فقط 4 ورودی
[outAvoid,~,~,~,rfA] = evalfis(fisAvoid, inAvoid);

voltage_left  = outAvoid(1);
voltage_right = outAvoid(2);

% --- ثبت آمار قواعد اجتناب ---
rfA = max(rfA,[],2);                 % اگر نوع‑۲ است یک ستونش کنید
ruleCountAvoid  = ruleCountAvoid  + (rfA > 0);
ruleEnergyAvoid = ruleEnergyAvoid + rfA;

else
    controlMode = 1;

    % --- FOPID heading (ψ) ---
    [e_psi,  z_int_psi] = filter(b_psi, a_psi, angleError, z_int_psi);
    output_psi = e_psi;

% --- خطای سرعت (یک‌بار در هر گام) ---
actualSpeed    = state(timeStep,13);
speedError_raw = desiredSpeed_now - actualSpeed;
eSpd_f         = eSpd_f + alpha_spd*(speedError_raw - eSpd_f);  % LPF
speedError_k   = eSpd_f;                                        % خطای سرعتِ صاف‌شده
speed_error_array(end+1) = abs(speedError_k);                   % برای شاخص‌ها
speedError = speedError_k;                                      % برای چاپ در انتهای گام

[e_u, z_int_u] = filter(b_u, a_u, speedError, z_int_u);
output_u = e_u;   % بدون کف صفر

% --- تخمین انحنای محلی از سه چک‌پوینت (prev, curr, next) ---
idx = counterCheckpoint;
i0  = max(1, idx-1);
i2  = min(size(checkpoints,1), idx+1);

p0  = checkpoints(i0 ,:);
pc  = checkpoints(idx,:);
p2  = checkpoints(i2 ,:);





kV = 0.5;           % به‌جای 0.5؛ اول با 1.0 تست کن
vL = kV*(output_u + output_psi);
vR = kV*(output_u - output_psi);

Vmax = 10;          % سقف فیزیکی مناسب درایو/موتورتان
voltage_left  = max(-Vmax, min(Vmax, vL));
voltage_right = max(-Vmax, min(Vmax, vR));


    angle_error_array(end+1) = abs(angleError);
end


% 
voltages = [voltage_left; voltage_left; voltage_right; voltage_right];


        % ذخیره مقادیر ولتاژ
    voltage_left_values(end+1) = voltage_left;
    voltage_right_values(end+1) = voltage_right;
    time_voltage(end+1) = (timeStep-1)*stepSize_time;  % زمان مربوط به این تکرار


        [dstate, state(timeStep,:)] = DynamicalModel(voltages, state(timeStep,:), stepSize_time);
        if timeStep < timeSteps_total
            state(timeStep+1,:) = state(timeStep,:) + dstate * stepSize_time;
        end
        
        % ذخیره مسیر
        path_x(end+1,1) = state(timeStep,19);
        path_y(end+1,1) = state(timeStep,20);
        
        % به‌روز‌رسانی تصویر آنلاین
        cla(fig);
        show(refMapDisp);  hold on; 
        plot(startPointSim(1), startPointSim(2), 'gx','MarkerSize',8,'LineWidth',2);
        text(startPointSim(1)+0.3, startPointSim(2)+0.3,...
            'Start','Color','green','FontSize',10,'FontWeight','bold');
        plot(checkpoints(:,1), checkpoints(:,2), 'rx--','LineWidth',1.5);
        plot(path_x, path_y, 'k--','LineWidth',1.5);
% ----------- رسم موانع متحرک -----------
for k = 1:numObs
    plot(movObs(k).x, movObs(k).y, movObs(k).plotArgs{:});
end


        DrawRobot(0.2, state(timeStep,19), state(timeStep,20), state(timeStep,24), 'm');
            title(sprintf('Mode=%d  |  d_{goal}=%.2f m  |  d_{obs}=%.2f m', ...
    controlMode, goalDist, minDistMovObs));


        pause(0.01);
    end


   
%% === گزارش و حذف قواعد کم‌استفاده برای کنترل اجتناب ===
energyThrAvoid   = 0.01;   % آستانهٔ ۱٪
unusedAvoid      = find(ruleCountAvoid == 0);
rareAvoid        = find( ruleEnergyAvoid/sum(ruleEnergyAvoid) < energyThrAvoid & ruleCountAvoid > 0);
rules2DelAvoid   = unique([unusedAvoid; rareAvoid]);

fprintf('\n--- Obstacle‑Avoidance Rule‑Pruning Report ---\n');
fprintf('Total avoid rules    : %d\n', numRulesAvoid);
fprintf('Unused avoid rules   : %s\n', mat2str(unusedAvoid'));
fprintf('Low‑energy (<1%%)     : %s\n', mat2str(setdiff(rareAvoid,unusedAvoid)'));

if ~isempty(rules2DelAvoid)
    fisAvoidPruned = fisAvoid;
    fisAvoidPruned.Rules(rules2DelAvoid) = [];  
    save('fisAvoid_pruned.mat','fisAvoidPruned');
    fprintf('Avoid rules removed  : %d → Remaining: %d\n', ...
            numel(rules2DelAvoid), numel(fisAvoidPruned.Rules));
else
    fprintf('No avoidance rules qualified for deletion.\n');
end

    elapsedControl = toc(tic_control);
    fprintf(' simulation time (from end of path planning to destination): %.2f sec\n', elapsedControl);

%% --- شاخص‌های خطای زاویه (همانِ قبلی شما) ---
timeError = (0:stepSize_time:(length(angle_error_array)-1)*stepSize_time);

MSE_angle  = mean(angle_error_array.^2);
RMSE_angle = sqrt(mean(angle_error_array.^2));
IAE_angle  = trapz(timeError, abs(angle_error_array));
ISE_angle  = trapz(timeError, angle_error_array.^2);
MAE_angle  = mean(abs(angle_error_array));

fprintf('\n--- شاخص های خطا ---\n');
fprintf('Angle Error Metrics:\n');
fprintf('MSE: %.4f\n', MSE_angle);
fprintf('IAE: %.4f\n', IAE_angle);
fprintf('ISE: %.4f\n', ISE_angle);
fprintf('MAE: %.4f\n', MAE_angle);
fprintf('RMS Error (angle): %.4f\n', RMSE_angle);

%% --- شاخص‌های خطای سرعت (جدید، عین زاویه) ---
timeSpeed = (0:stepSize_time:(length(speed_error_array)-1)*stepSize_time);

MSE_speed  = mean(speed_error_array.^2);
RMSE_speed = sqrt(mean(speed_error_array.^2));
IAE_speed  = trapz(timeSpeed, abs(speed_error_array));
ISE_speed  = trapz(timeSpeed, speed_error_array.^2);
MAE_speed  = mean(abs(speed_error_array));

fprintf('\nSpeed Error Metrics:\n');
fprintf('MSE: %.4f\n', MSE_speed);
fprintf('IAE: %.4f\n', IAE_speed);
fprintf('ISE: %.4f\n', ISE_speed);
fprintf('MAE: %.4f\n', MAE_speed);
fprintf('RMS Error (speed): %.4f\n', RMSE_speed);

    % محاسبه مسافت پیموده شده
    totalDist = 0;
    for i = 2:length(path_x)
        dx = path_x(i) - path_x(i-1);
        dy = path_y(i) - path_y(i-1);
        totalDist = totalDist + sqrt(dx^2 + dy^2);
    end
    absoluteErr      = abs(totalDist - pathLengthOpt);            % خطای مطلق [m]
percentErr       = (absoluteErr / pathLengthOpt) * 100;       % خطای درصدی [%]

fprintf('\n--- مقایسۀ طول مسیر ---\n');
fprintf('Planned Length : %.3f m\n', pathLengthOpt);
fprintf('Actual  Length : %.3f m\n', totalDist);
fprintf('Absolute Error : %.3f m\n', absoluteErr);
fprintf('Percent  Error : %.2f %%\n', percentErr);

   %% نمودار مسیریابی نهایی
    figure('Name','Final Path Planning and Robot Trajectory');
    show(refMapDisp); hold on;
    plot(gpop.info.xx, gpop.info.yy, 'r-', 'LineWidth', 2);
    plot(checkpoints(:,1), checkpoints(:,2), 'bo', 'MarkerSize',8, 'LineWidth',2);
    plot(path_x, path_y, 'k--', 'LineWidth', 1.5);
    plot(startPointSim(1), startPointSim(2), 'gx', 'MarkerSize',10, 'LineWidth',2);
    plot(finalGoalSim(1), finalGoalSim(2), 'mx', 'MarkerSize',10, 'LineWidth',2);
    legend('Dragonfly Spline Path','Checkpoints','Robot Path','Start','Goal','Location','best');
    title('Final Path Planning and Robot Trajectory');
    xlabel('X [m]'); ylabel('Y [m]');
    grid on;
    
    %% رسم نمودار همگرایی Dragonfly (Best Cost vs. Iteration)
    figure('Name','Dragonfly Convergence');
    plot(bestCostHistory, 'LineWidth', 2, 'Color', 'b');
    xlabel('Iteration');
    ylabel('Best Cost');
    title('Dragonfly Convergence Plot');
    grid on;
figure('Name','Voltages (Left / Right)');
subplot(2,1,1);
plot(time_voltage, voltage_left_values, 'LineWidth', 1.6); grid on;
title('Left Voltage'); xlabel('Time (s)'); ylabel('V');

subplot(2,1,2);
plot(time_voltage, voltage_right_values, 'LineWidth', 1.6); grid on;
title('Right Voltage'); xlabel('Time (s)'); ylabel('V');

linkaxes(findall(gcf,'Type','axes'),'x');   % همگام‌سازی محور زمان
% ==== Align lengths safely ====
Nstate = size(state,1);          % تعداد نمونه‌های ذخیره شده در state
Ntime  = numel(time_voltage);    % تعداد نمونه‌های بردار زمان/ولتاژ
N      = min(Nstate, Ntime);     % حداقل طول مشترک

% اگر time_voltage خالی بود، زمان را از روی گام زمانی بساز
if Ntime == 0
    t_vec = (0:Nstate-1) * stepSize_time;
    N = Nstate;
else
    t_vec = time_voltage(1:N);
end

u_vec = state(1:N,13);           % u: سرعت روبه‌جلو (m/s)
v_vec = state(1:N,14);           % v: سرعت جانبی (m/s)
r_vec = state(1:N,18);           % r: سرعت زاویه‌ای حول z (rad/s)

lin_forward = u_vec;             
lin_speed   = hypot(u_vec, v_vec);
yaw_rate    = r_vec;

% === Speeds in ONE window, separate subplots (3x1) ===
figure('Name','Speeds (u, |v|, r) — separate');
subplot(3,1,1);
plot(t_vec, lin_forward, 'LineWidth', 1.8); grid on;
title('Linear Speed u (forward)'); xlabel('Time (s)'); ylabel('m/s');

subplot(3,1,2);
plot(t_vec, lin_speed, 'LineWidth', 1.8); grid on;
title('Planar Speed Magnitude |v|'); xlabel('Time (s)'); ylabel('m/s');

subplot(3,1,3);
plot(t_vec, yaw_rate, 'LineWidth', 1.8); grid on;
title('Yaw Rate r'); xlabel('Time (s)'); ylabel('rad/s');

% همگام‌سازی محور زمان بین هر سه نمودار
linkaxes(findall(gcf,'Type','axes'),'x');

% گشتاور چرخ‌ها از state
t1 = state(1:N, 3);   % چرخ 1 (چپ جلو)
t2 = state(1:N, 6);   % چرخ 2 (چپ عقب)
t3 = state(1:N, 9);   % چرخ 3 (راست جلو)
t4 = state(1:N,12);   % چرخ 4 (راست عقب)


% — نمودار 2×2 جداگانه
figure('Name','Wheel Torques (2x2)');
subplot(2,2,1); plot(t_vec, t1, 'LineWidth', 1.6); grid on; title('Wheel 1 (LF)'); xlabel('Time (s)'); ylabel('Nm');
subplot(2,2,2); plot(t_vec, t2, 'LineWidth', 1.6); grid on; title('Wheel 2 (LR)'); xlabel('Time (s)'); ylabel('Nm');
subplot(2,2,3); plot(t_vec, t3, 'LineWidth', 1.6); grid on; title('Wheel 3 (RF)'); xlabel('Time (s)'); ylabel('Nm');
subplot(2,2,4); plot(t_vec, t4, 'LineWidth', 1.6); grid on; title('Wheel 4 (RR)'); xlabel('Time (s)'); ylabel('Nm');


% === Wheel Angular Speeds (w1..w4) in ONE window (2x2) ===

% === Wheel Angular Speeds (w1..w4) in ONE window (2x2) ===
w1 = state(1:N, 2);   % Wheel 1 (LF)
w2 = state(1:N, 5);   % Wheel 2 (LR)
w3 = state(1:N, 8);   % Wheel 3 (RF)
w4 = state(1:N,11);   % Wheel 4 (RR)

figure('Name','Wheel Angular Speeds (rad/s) — 2x2');
subplot(2,2,1);
plot(t_vec, w1, 'LineWidth', 1.6); grid on;
title('Wheel 1 (LF) \omega_1'); xlabel('Time (s)'); ylabel('rad/s');

subplot(2,2,2);
plot(t_vec, w2, 'LineWidth', 1.6); grid on;
title('Wheel 2 (LR) \omega_2'); xlabel('Time (s)'); ylabel('rad/s');

subplot(2,2,3);
plot(t_vec, w3, 'LineWidth', 1.6); grid on;
title('Wheel 3 (RF) \omega_3'); xlabel('Time (s)'); ylabel('rad/s');

subplot(2,2,4);
plot(t_vec, w4, 'LineWidth', 1.6); grid on;
title('Wheel 4 (RR) \omega_4'); xlabel('Time (s)'); ylabel('rad/s');

linkaxes(findall(gcf,'Type','axes'),'x');   % همگام‌سازی محور زمان
% === Speedometer: scalar speed (m/s) ===
if N > 0
    % سرعت صفحه‌ای در دستگاه بدنه (m/s)
    speed_mps = hypot(u_vec, v_vec);

    % (اختیاری) صاف‌سازی نمای سرعت برای خوانایی (پنجره ~0.5 ثانیه)
    win = max(1, round(0.5/stepSize_time));
    speed_mps_smooth = movmean(speed_mps, win);

    % نمودار سرعت یک‌عددی
    figure('Name','Robot Speed (m/s)');
    plot(t_vec, speed_mps_smooth, 'LineWidth', 1.8); grid on;
    xlabel('Time (s)'); ylabel('Speed (m/s)');
    title('Robot Speed (scalar)');

    % نمایش عدد آخر روی نمودار
    last_speed = speed_mps_smooth(end);
    text(t_vec(end), last_speed, sprintf('  %.2f m/s', last_speed), ...
         'VerticalAlignment','bottom','FontWeight','bold');
end
%% === Heading ψ (yaw), X و Y بر حسب زمان (همه در یک پنجره) ===
x_vec   = state(1:N,19);       % x [m]
y_vec   = state(1:N,20);       % y [m]
psi_vec = state(1:N,24);       % ψ (yaw) [rad]
psi_unw = unwrap(psi_vec);     % باز کردن پرش‌های ±pi
% اگر درجه می‌خوای: psi_plot = rad2deg(psi_unw);  ylabel('\psi (deg)')
psi_plot = psi_unw;

figure('Name','Yaw ψ, X, Y vs Time');
subplot(3,1,1);
plot(t_vec, psi_plot, 'LineWidth', 1.8); grid on;
title('Yaw (Heading) \psi'); xlabel('Time (s)'); ylabel('\psi (rad)');

subplot(3,1,2);
plot(t_vec, x_vec, 'LineWidth', 1.8); grid on;
title('X position'); xlabel('Time (s)'); ylabel('x (m)');

subplot(3,1,3);
plot(t_vec, y_vec, 'LineWidth', 1.8); grid on;
title('Y position'); xlabel('Time (s)'); ylabel('y (m)');

linkaxes(findall(gcf,'Type','axes'),'x');  % همگام‌سازی محور زمان

% ==== PACK & SAVE: همه‌ی خروجی‌های مربوط به کنترلر فازی در یک فایل ====
label = 'FOPID2m2';  % <-- برای هر کنترلر تغییر بده

% هم‌ترازسازی طول‌ها بر اساس N (که قبلاً محاسبه کردی)
idxN = 1:N;

% ساختار نتیجه
res = struct();

% فراداده
res.meta.label       = label;
res.meta.controller  = 'fuzzy-main';      % اگر دوست داری نوع را دقیق‌تر بنویس
res.meta.avoid_ctrl  = 'fuzzy-avoid';     % اگر اجتناب فازی استفاده می‌کنی
res.meta.stepSize    = stepSize_time;
res.meta.note        = 'all fuzzy-controller related signals (one run)';

% زمان‌ها
res.time.main        = t_vec(:);                      % محور مشترک مشتق از state/time_voltage
res.time.volt        = time_voltage(:);               % محور ثبت ولتاژ (خام)

% ولتاژها (تا N)
res.voltage.left     = voltage_left_values(idxN).';
res.voltage.right    = voltage_right_values(idxN).';

% سرعت‌ها از state (تا N)
res.speed.u_forward  = u_vec(idxN).';
res.speed.v_lateral  = v_vec(idxN).';
res.speed.yaw_rate_r = r_vec(idxN).';
res.speed.planar_mag = lin_speed(idxN).';

% گشتاور چرخ‌ها (تا N)
res.torque.w1_LF     = t1(idxN).';
res.torque.w2_LR     = t2(idxN).';
res.torque.w3_RF     = t3(idxN).';
res.torque.w4_RR     = t4(idxN).';

% سرعت زاویه‌ای چرخ‌ها (تا N)
res.omega.w1_LF      = w1(idxN).';
res.omega.w2_LR      = w2(idxN).';
res.omega.w3_RF      = w3(idxN).';
res.omega.w4_RR      = w4(idxN).';

% سرعت اسکالر صاف‌شده (تا N و با محافظت)
if exist('speed_mps_smooth','var') && numel(speed_mps_smooth) >= N
    res.speed.scalar_smooth = speed_mps_smooth(idxN).';
else
    res.speed.scalar_smooth = hypot(u_vec(idxN), v_vec(idxN)).';
end

% وضعیت/پوز ربات (تا N)
res.pose.x           = x_vec(idxN).';
res.pose.y           = y_vec(idxN).';
res.pose.psi_rad     = psi_plot(idxN).';  % yaw unwrap شده

% مسیر طی‌شده و مسیر برنامه‌ریزی شده (برای ترسیم‌های بعدی)
if exist('path_x','var') && ~isempty(path_x)
    res.path.traj_xy = [path_x(:), path_y(:)];
end
if exist('checkpoints','var') && ~isempty(checkpoints)
    res.path.checkpoints = checkpoints;
end
if exist('gpop','var') && isfield(gpop,'info')
    res.path.dragonfly_x = gpop.info.xx(:);
    res.path.dragonfly_y = gpop.info.yy(:);
end

% ذخیره در فایل .mat
save(sprintf('run_%s.mat', label), 'res');
fprintf('Saved FOPID2m2-run pack -> run_%s.mat\n', label);

    elapsedTotalTime = toc(tic_total);
    fprintf('Total simulation time: %.2f sec\n', elapsedTotalTime);
    
end



%% ========================================================================
%% تابع مسیر‌یابی با الگوریتم دراگون‌فلای (Dragonfly)
%% ========================================================================
function [gpop, bestCost, bestCostHistory] = DragonflyPathPlanning_DF(data)
    global NFE
    NFE = 0;
    
    % پارامترهای الگوریتم
    npop = 100;        % اندازه جمعیت
    maxiter = 110;    % تعداد تکرار
     
    % تعیین شعاع اولیه همسایگی و حداکثر تغییرات
    r = (data.UB - data.LB) / 10;
    Delta_max = (data.UB - data.LB) / 10;
    
    % تعریف ساختار پایه (با تمام فیلدهای لازم)
    emp = struct('x', [], 'fit', [], 'info', [], 'SCH', [], 'L', [], 'Violation', [], 'IsFeasible', []);
    pop = repmat(emp, npop, 1);
    Deltapop = repmat(emp, npop, 1);
    
    % مقداردهی اولیه جمعیت (راه‌حل‌های تصادفی بین LB و UB)
    for i = 1:npop
        pop(i).x = unifrnd(data.LB, data.UB);
        pop(i) = fitness_DF(pop(i), data);
    end
    for i = 1:npop
        Deltapop(i).x = unifrnd(data.LB, data.UB);
        Deltapop(i) = fitness_DF(Deltapop(i), data);
    end
    
    % تعیین بهترین (gpop) و بدترین (epop) راه حل
    [~, ind] = min([pop.fit]); 
    gpop = pop(ind);
    [~, ind] = max([pop.fit]); 
    epop = pop(ind);
    
    bestCostHistory = zeros(maxiter,1);
    
    % حلقه اصلی الگوریتم دراگون‌فلای
    for iter = 1:maxiter
        % تنظیم شعاع حسی به صورت خطی
        r = (data.UB - data.LB) / 4 + ((data.UB - data.LB) * (iter/maxiter) * 2);
        w = 0.9 - iter*((0.9-0.4)/maxiter);
        my_c = 0.1 - iter*((0.1-0)/(maxiter/2));
        if my_c < 0, my_c = 0; end
        
        s = 2 * rand * my_c;  % وزن جداسازی
        a = 2 * rand * my_c;  % وزن همسویی
        c = 2 * rand * my_c;  % وزن تجانس
        f = 2 * rand;         % وزن جذب به سمت غذا (بهترین)
        e = my_c;             % وزن دوری از دشمن (بدترین)
        
        for i = 1:npop
            index = 0;
            neighbours_no = 0;
            Neighbours_DeltaX = [];
            Neighbours_X = [];
            % یافتن همسایگان درون شعاع r
            for j = 1:npop
                Dist2Enemy = distance(pop(i).x, pop(j).x);
                if (all(Dist2Enemy <= r) && any(Dist2Enemy > 0))
                    index = index + 1;
                    neighbours_no = neighbours_no + 1;
                    Neighbours_DeltaX(:, index) = Deltapop(j).x(:);
                    Neighbours_X(:, index) = pop(j).x(:);
                end
            end
            
            % جداسازی: جلوگیری از تراکم
            S = zeros(length(pop(i).x), 1);
            if neighbours_no > 1
                for k = 1:neighbours_no
                    S = S + (Neighbours_X(:, k) - pop(i).x(:));
                end
                S = -S;
            else
                S = zeros(length(pop(i).x), 1);
            end
            
            % همسویی: تطبیق سرعت
            if neighbours_no > 1
                A = mean(Neighbours_DeltaX, 2);
            else
                A = Deltapop(i).x(:);
            end
            
            % تجانس: حرکت به سمت مرکز همسایگان
            if neighbours_no > 1
                C_temp = mean(Neighbours_X, 2);
            else
                C_temp = pop(i).x(:);
            end
            C = C_temp - pop(i).x(:);
            
            % جذب به سمت بهترین (غذا)
            Dist2Food = distance(pop(i).x, gpop.x);
            if all(Dist2Food <= r)
                F = gpop.x(:) - pop(i).x(:);
            else
                F = zeros(size(gpop.x(:)));
            end
            
            % دوری از بدترین (دشمن)
            Dist2Enemy = distance(pop(i).x, epop.x);
            if all(Dist2Enemy <= r)
                Enemy = epop.x(:) + pop(i).x(:);
            else
                Enemy = zeros(size(gpop.x(:)));
            end
            
            % بررسی کران‌ها (wrap-around)
            for tt = 1:length(pop(i).x)
                if pop(i).x(tt) > data.UB(tt)
                    pop(i).x(tt) = data.LB(tt);
                end
                if pop(i).x(tt) < data.LB(tt)
                    pop(i).x(tt) = data.UB(tt);
                end
            end
            
            % به‌روزرسانی سرعت و موقعیت
            if any(Dist2Food > r)
                if neighbours_no > 1
                    for j = 1:length(pop(i).x)
                        Deltapop(i).x(j) = w * Deltapop(i).x(j) + rand * A(j) + rand * C(j) + rand * S(j);
                        if Deltapop(i).x(j) > Delta_max(j)
                            Deltapop(i).x(j) = Delta_max(j);
                        end
                        if Deltapop(i).x(j) < -Delta_max(j)
                            Deltapop(i).x(j) = -Delta_max(j);
                        end
                        pop(i).x(j) = pop(i).x(j) + Deltapop(i).x(j);
                    end
                else
                    pop(i).x = pop(i).x + Levy(length(pop(i).x)) .* pop(i).x;
                end
            else
                for j = 1:length(pop(i).x)
                    Deltapop(i).x(j) = (a * A(j) + c * C(j) + s * S(j) + f * F(j) + e * epop.x(j)) + w * Deltapop(i).x(j);
                    if Deltapop(i).x(j) > Delta_max(j)
                        Deltapop(i).x(j) = Delta_max(j);
                    end
                    if Deltapop(i).x(j) < -Delta_max(j)
                        Deltapop(i).x(j) = -Delta_max(j);
                    end
                    pop(i).x(j) = pop(i).x(j) + Deltapop(i).x(j);
                end 
            end
            
            % ارزیابی موقعیت جدید با استفاده از تابع fitness
            pop(i) = fitness_DF(pop(i), data);
        end
        
        % به‌روزرسانی بهترین و بدترین راه حل
        [val1, ind1] = min([pop.fit]);
        if val1 < gpop.fit
            gpop = pop(ind1);
        end
        [val2, ind2] = max([pop.fit]);
        if val2 > epop.fit
            epop = pop(ind2);
        end
        
        bestCostHistory(iter) = gpop.fit;
        disp(['DA Iter = ' num2str(iter) ', Best Cost = ' num2str(bestCostHistory(iter))]);
    end
    
    bestCost = gpop.fit;
end

%% ========================================================================
%% تابع Fitness برای دراگون‌فلای (استفاده از occupancy map)
%% ========================================================================
function sol = fitness_DF(sol, data)
    % بردار تصمیم: n مقدار x به دنبال n مقدار y
    A = sol.x;
    n = data.n;
    
    % ساخت مسیر کامل از (شروع -> نقاط میانی -> هدف)
    XS = [data.xs, A(1:n), data.xt];
    YS = [data.ys, A(n+1:end), data.yt];
    k = length(XS);
    
    TS = linspace(0,1,k);
    tt = linspace(0,1,500);  % رزولوشن اینترپولاسیون اسپلاین
    xx = spline(TS, XS, tt);
    yy = spline(TS, YS, tt);
    
    dx = diff(xx);
    dy = diff(yy);
    L = sum(sqrt(dx.^2+dy.^2));
    
    % بررسی برخورد با موانع با استفاده از occupancy map
    collisionCount = 0;
    for i = 1:length(xx)
        if checkOccupancy(data.refMap, [xx(i), yy(i)])   % ← به جای refMapDisp از refMap استفاده کن

            collisionCount = collisionCount + 1;
        end
    end
    Violation = collisionCount / length(xx);
    
    beta = data.beta;
       % --- ۱) Proximity Penalty ---
    % از data.distanceMap که در کد اصلی دارید استفاده می‌کنیم
    % نمونه‌برداری در 100 نقطه‌ی مساوی از spline
    idx = round(linspace(1, numel(xx), 100));
    ds = zeros(1, numel(idx));
    for j = 1:numel(idx)
        pt = [xx(idx(j)), yy(idx(j))];
        ds(j) = getDistanceToObstacle(pt, data.refMap, data.distanceMap);  % ← refMapDisp → refMap
    end

    d_bar = mean(ds);
    
    clearanceThr = data.clearanceThr;  % مثلاً 0.5 متر
    alpha       = data.wProx;          % وزن جریمه (مثلاً 500)
    proxPenalty = alpha * max(0, clearanceThr - d_bar);

    % --- جمع نهایی هزینه ---
    sol.fit = L * (1 + beta * Violation) + proxPenalty;

    sol.info.xx = xx;
    sol.info.yy = yy;
    sol.L = L;
    sol.Violation = Violation;
    sol.IsFeasible = (Violation == 0);
end

%% ========================================================================
%% تابع Levy Flight
%% ========================================================================
function o = Levy(d)
    beta = 3/2;
    sigma = ( gamma(1+beta)*sin(pi*beta/2) / ( gamma((1+beta)/2)*beta*2^((beta-1)/2) ) )^(1/beta);
    u = randn(1,d)*sigma;
    v = randn(1,d);
    step = u ./ abs(v).^(1/beta);
    o = 0.01 * step;
end

%% ========================================================================
%% تابع فاصله (Euclidean Distance)
%% ========================================================================
function o = distance(a, b)
    % فاصله به صورت اختلاف مطلق هر مولفه
    o = abs(a - b);
end
%% =========================================================================
%% بهینه‌سازی موضعی (Local Path Optimization)
%% =========================================================================
function [xx_opt, yy_opt] = localPathOptimization(xx, yy, refMap, distanceMap)
    alpha =1;   % گام کوچک‌تر برای اصلاح دقیق‌تر
    maxIter = 80;   % تکرار بیشتر
    threshold = 500; % آستانهٔ هزینه جهت شروع اصلاح

    numPoints = length(xx);
    xx_opt = xx;
    yy_opt = yy;

    for i = 1:numPoints
        point = [xx_opt(i), yy_opt(i)];
        cost_pt = localCost(point, refMap, distanceMap);
        % اگر هزینه بالاست، تلاش برای اصلاح
        if cost_pt > threshold
            currentPoint = point;
            localAlpha = alpha;
            for iter = 1:maxIter
                grad = estimateGradient(currentPoint, refMap, distanceMap);
                newPoint = currentPoint - localAlpha * grad;
                oldCost = localCost(currentPoint, refMap, distanceMap);
                newCost = localCost(newPoint, refMap, distanceMap);
                if newCost < oldCost
                    currentPoint = newPoint;
                else
                    localAlpha = localAlpha * 0.5;
                    if localAlpha < 1e-5
                        break;
                    end
                end
            end
            xx_opt(i) = currentPoint(1);
            yy_opt(i) = currentPoint(2);
        end
    end
end

function cost = localCost(point, refMap, distanceMap)
    % اگر داخل مانع هستیم، هزینهٔ بی‌نهایت
    if checkOccupancy(refMap, point)
        cost = inf;  
    else
        d = getDistanceToObstacle(point, refMap, distanceMap);
         cost = 1/(d^2 + eps) + 10*exp(-5*d);
    end
end

function grad = estimateGradient(point, refMap, distanceMap)
    delta = 1e-3;
    cost_x1 = localCost([point(1)+delta, point(2)], refMap, distanceMap);
    cost_x2 = localCost([point(1)-delta, point(2)], refMap, distanceMap);
    grad_x = (cost_x1 - cost_x2) / (2*delta);

    cost_y1 = localCost([point(1), point(2)+delta], refMap, distanceMap);
    cost_y2 = localCost([point(1), point(2)-delta], refMap, distanceMap);
    grad_y = (cost_y1 - cost_y2) / (2*delta);

    grad = [grad_x, grad_y];
end

function d = getDistanceToObstacle(point, refMap, distanceMap)
    xWorld = point(1);
    yWorld = point(2);

    xLimits = refMap.XWorldLimits;
    yLimits = refMap.YWorldLimits;
    resolution = refMap.Resolution;

    col = round((xWorld - xLimits(1)) * resolution) + 1;
    row = size(distanceMap,1) - round((yWorld - yLimits(1)) * resolution);

    row = max(min(row, size(distanceMap,1)), 1);
    col = max(min(col, size(distanceMap,2)), 1);

    d = distanceMap(row, col);
end
