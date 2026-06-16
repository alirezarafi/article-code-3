function fisAvoid = a107()
    % Generate new Type-2 fuzzy inference system
    fisAvoid = mamfistype2('Name', "turning_angle_fl_type2", 'AndMethod', "prod", 'DefuzzificationMethod', "centroid");
fisAvoid.TypeReductionMethod  = "ekm";   % یا "enhancedKarnikMendel"
    % Add inputs
    maxObs = 20;
    fisAvoid = addInput(fisAvoid, [-400 400], 'Name', "turning_angle");
    fisAvoid = addInput(fisAvoid,[0 maxObs],'Name',"FL");
    fisAvoid = addInput(fisAvoid,[0 maxObs],'Name',"F");
    fisAvoid = addInput(fisAvoid,[0 maxObs],'Name',"FR");

    % Add outputs
    fisAvoid = addOutput(fisAvoid, [-15 15], 'Name', "left_voltage");
    fisAvoid = addOutput(fisAvoid, [-15 15], 'Name', "right_voltage");

    % Add membership functions for turning_angle (as Type-2)
fisAvoid.Inputs(1).MembershipFunctions(1) = fismftype2('trapmf',[-400 -60 -15 -5],'Name','Negativeturn');
fisAvoid.Inputs(1).MembershipFunctions(2) = fismftype2('trimf',[-8 0 8],'Name','Threshold');   % حول صفر، باریک و دقیق
fisAvoid.Inputs(1).MembershipFunctions(3) = fismftype2('trapmf',[5 15 60 400],'Name','Positiveturn');

    % Add membership functions for FL (as Type-2)
  fisAvoid.Inputs(2).MembershipFunctions(1) = fismftype2('trapmf',[0 0 0.4 1],'Name','Close');
fisAvoid.Inputs(2).MembershipFunctions(2) = fismftype2('trimf' ,[0.9 1.6 3.5],'Name','Medium');
fisAvoid.Inputs(2).MembershipFunctions(3) = fismftype2('trapmf',[2.8 5 maxObs maxObs],'Name','Far');

   % Add membership functions for F (as Type-2)
   fisAvoid.Inputs(3).MembershipFunctions(1) = fismftype2('trapmf',[0 0 0.4 1] ,'Name','Close');  % قبلی: [0 0 0.5 2]
fisAvoid.Inputs(3).MembershipFunctions(2) = fismftype2('trimf' ,[0.9 1.6 3.5],'Name','Medium');     % قبلی: [1.6 2 6]
fisAvoid.Inputs(3).MembershipFunctions(3) = fismftype2('trapmf',[2.8 5 maxObs maxObs],'Name','Far'); % قبلی: [3 6 ...]

   % Add membership functions for FR (as Type-2)
   fisAvoid.Inputs(4).MembershipFunctions(1) = fismftype2('trapmf',[0 0 0.4 1],'Name','Close');
fisAvoid.Inputs(4).MembershipFunctions(2) = fismftype2('trimf' ,[0.9 1.6 3.5],'Name','Medium');
fisAvoid.Inputs(4).MembershipFunctions(3) = fismftype2('trapmf',[2.8 5 maxObs maxObs],'Name','Far');

 
    % Add membership functions for left_voltage (as Type-2)
fisAvoid.Outputs(1).MembershipFunctions(1) = fismftype2('trapmf',[-10 -8 -3 -1.5],'Name','Strong_Reverse');
    fisAvoid.Outputs(1).MembershipFunctions(2) = fismftype2('trimf', [-5 -2.5 0], 'Name', 'Reverse');
    fisAvoid.Outputs(1).MembershipFunctions(3) = fismftype2('trimf', [-2.5 0 2.5], 'Name', 'Neutral');
    fisAvoid.Outputs(1).MembershipFunctions(4) = fismftype2('trimf', [0 5 12], 'Name', 'Forward');
fisAvoid.Outputs(1).MembershipFunctions(5) = fismftype2('trimf',[10 11 12],'Name','Strong_Forward');

    % Add membership functions for right_voltage (as Type-2)
fisAvoid.Outputs(2).MembershipFunctions(1) = fismftype2('trapmf',[-10 -8 -3 -1.5],'Name','Strong_Reverse');
    fisAvoid.Outputs(2).MembershipFunctions(2) = fismftype2('trimf', [-5 -2.5 0], 'Name', 'Reverse');
    fisAvoid.Outputs(2).MembershipFunctions(3) = fismftype2('trimf', [-2.5 0 2.5], 'Name', 'Neutral');
    fisAvoid.Outputs(2).MembershipFunctions(4) = fismftype2('trimf', [0 5 12], 'Name', 'Forward');
fisAvoid.Outputs(2).MembershipFunctions(5) = fismftype2('trimf',[10 11 12],'Name','Strong_Forward');


    % Define the rules
    rules = [

% --- جایگزین: فرمان‌دهی ملایمِ همسو با هدف وقتی فضا باز است
"if F is Far and turning_angle is Positiveturn then left_voltage  is Neutral (0.6)"
"if F is Far and turning_angle is Positiveturn then right_voltage is Forward (0.9)"
"if F is Far and turning_angle is Negativeturn then left_voltage  is Forward (0.9)"
"if F is Far and turning_angle is Negativeturn then right_voltage is Neutral (0.6)"
"if F is Far and turning_angle is Threshold     then left_voltage  is Forward (0.8)"
"if F is Far and turning_angle is Threshold     then right_voltage is Forward (0.8)"
 
   % ==== راهروی باریک (هر دو طرف Medium) ====
    "if FL is Medium and FR is Medium and F is Far and turning_angle is Threshold     then left_voltage  is Forward (0.6)"
    "if FL is Medium and FR is Medium and F is Far and turning_angle is Threshold     then right_voltage is Forward (0.6)"
    "if FL is Medium and FR is Medium and F is Far and turning_angle is Positiveturn then left_voltage  is Neutral (0.5)"
    "if FL is Medium and FR is Medium and F is Far and turning_angle is Positiveturn then right_voltage is Forward (0.7)"
    "if FL is Medium and FR is Medium and F is Far and turning_angle is Negativeturn then left_voltage  is Forward (0.7)"
    "if FL is Medium and FR is Medium and F is Far and turning_angle is Negativeturn then right_voltage is Neutral (0.5)"

% هر دو طرف خیلی نزدیک و جلو باز نیست → توقف کامل
    "if FL is Close and FR is Close    then left_voltage  is Neutral (1)"
    "if FL is Close and FR is Close     then right_voltage is Neutral (1)"
    
        % توقف اضطراری جلو
        "if F is Close then left_voltage  is Neutral (1)"
        "if F is Close then right_voltage is Neutral (1)"
    ];


    fisAvoid = addRule(fisAvoid, rules);
   end
% %% ===== رسم توابع عضویت ورودی‌های fisAvoid =====
% for inIdx = 1:4          % 4 تا ورودی: turning_angle, FL, F, FR
%     figure;
%     plotmf(fisAvoid,'input',inIdx);
%     ax = gca;
%     title(ax, '');   % یا: ax.Title.String = '';
% 
%     % خطوط ضخیم‌تر
%     set(findall(ax,'Type','line'),'LineWidth',2.8);
% 
%     % رنگ ناحیه عدم قطعیت (FOU) = سبز خودش
%     hPatch = findobj(ax,'Type','patch');
%     set(hPatch, 'FaceColor', [0 0.7 0], ...   % سبز
%                 'FaceAlpha', 0.5, ...
%                 'EdgeColor', 'k');
% 
%     % لیبل‌های روی ممبرشیپ‌ها: بزرگ و بولد
%     txt = findall(ax,'Type','text');
%     set(txt, 'FontSize', 28, ...             % فقط متن روی MFها
%              'FontWeight','bold');
% 
%     % جابه‌جایی ویژه برای بعضی لیبل‌ها
%     if inIdx == 1
%         % ورودی 1: turning_angle → negative / Threshold / positive
%         for k = 1:numel(txt)
%             s   = string(txt(k).String);
%             pos = txt(k).Position;
% 
%             if contains(s,"negative","IgnoreCase",true)
%                 pos(1) = pos(1) - 56;
%             elseif contains(s,"Threshold","IgnoreCase",true)
%                 pos(2) = pos(2) + 0.08;
%             elseif contains(s,"positive","IgnoreCase",true)
%                 pos(1) = pos(1) + 56;
%             end
% 
%             txt(k).Position            = pos;
%             txt(k).HorizontalAlignment = 'center';
%             txt(k).VerticalAlignment   = 'bottom';
%         end
%     else
%         % ورودی‌های 2,3,4: Close / Medium / Far
%         for k = 1:numel(txt)
%             s   = string(txt(k).String);
%             pos = txt(k).Position;
% 
%             if contains(s,"Close","IgnoreCase",true)
%                 pos(1) = pos(1) + 0.6;
%             elseif contains(s,"Medium","IgnoreCase",true)
%                 pos(1) = pos(1) + 3;
%             end
% 
%             txt(k).Position            = pos;
%             txt(k).HorizontalAlignment = 'center';
%             txt(k).VerticalAlignment   = 'bottom';
%         end
%     end
% 
%     % تنظیم برچسب محور x (مثل قبل)
%     ax.XLabel.Units = 'normalized';
%     posXL = ax.XLabel.Position;
%     if inIdx == 1
%         posXL(1) = posXL(1) - 0.01;
%     else
%         posXL(1) = posXL(1) - 0.05;
%     end
%     posXL(2) = -0.14;
%     ax.XLabel.Position = posXL;
% 
%    % فونت محور و عنوان
%     ax.FontSize   = 28;
%     ax.FontWeight = 'bold';
%     ax.Title.FontSize   = 45;
%     ax.Title.FontWeight = 'bold';
%     ax.XLabel.FontSize  = 40;
%     ax.YLabel.FontSize  = 40;
% end
% 
% %% ===== رسم توابع عضویت خروجی‌های fisAvoid =====
% for outIdx = 1:2         % 2 تا خروجی: left_voltage, right_voltage
%     figure;
%     plotmf(fisAvoid,'output',outIdx);
%     ax = gca;
%     title(ax, '');   % یا: ax.Title.String = '';
% 
%     % خطوط ضخیم‌تر
%     set(findall(ax,'Type','line'),'LineWidth',3.5);
% 
%     % رنگ ناحیه عدم قطعیت (FOU) = سبز خودش
%     hPatch = findobj(ax,'Type','patch');
%     set(hPatch, 'FaceColor', [0 0.7 0], ...
%                 'FaceAlpha', 0.5, ...
%                 'EdgeColor', 'k');
% 
%     % لیبل‌های روی ممبرشیپ‌ها
%     txt = findall(ax,'Type','text');
%     set(txt, 'FontSize', 28, ...
%              'FontWeight','bold');
% 
% for k = 1:numel(txt)
%     s   = string(txt(k).String);
%     pos = txt(k).Position;
% 
%     % Strong_Reverse را کمی به چپ ببَر
%     if contains(s,"Strong_Reverse","IgnoreCase",true)
%         pos(1) = pos(1) - 3;
% 
%     % Strong_Forward را هم کمی به چپ ببَر که از کادر نزنه بیرون
%     elseif contains(s,"Strong_Forward","IgnoreCase",true)
%         pos(1) = pos(1) - 3.5;   % اگر هنوز بیرونه، بکنش -2 یا -2.5
% 
%     % Reverse (بدون Strong) را کمی به چپ ببَر
%     elseif contains(s,"Reverse","IgnoreCase",true) && ...
%            ~contains(s,"Strong","IgnoreCase",true)
%         pos(1) = pos(1) - 0.5;
% 
%     % Neutral را ببَر به راست
%     elseif contains(s,"Neutral","IgnoreCase",true)
%         pos(1) = pos(1) + 1;
%     end
% 
%     txt(k).Position            = pos;
%     txt(k).HorizontalAlignment = 'center';
%     txt(k).VerticalAlignment   = 'bottom';
% end
% 
% 
%     % برچسب محور x
%     ax.XLabel.Units = 'normalized';
%     posXL = ax.XLabel.Position;
%     posXL(1) = posXL(1) - 0;
%     posXL(2) = -0.135;
%     ax.XLabel.Position = posXL;
% 
%     % فونت محور و عنوان
%     ax.FontSize   = 28;
%     ax.FontWeight = 'bold';
%     ax.Title.FontSize   = 45;
%     ax.Title.FontWeight = 'bold';
%     ax.XLabel.FontSize  = 40;
%     ax.YLabel.FontSize  = 40;
% end
% 
