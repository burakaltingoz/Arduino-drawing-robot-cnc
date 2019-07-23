function cannyedgedet ()
    close all;  % hepsini kapatma
    sigma = 1.4; % Gaussian filter sigma
    highThresholdRatio = 0.2; % en yüksek eþik oraný
    lowThresholdRatio = 0.15; % en düþük eþik oraný
    
    % geçerli dosyayý m uzantýlý file yapmak için. çok gerek yok
    
    if(~isdeployed)
      cd(fileparts(which(mfilename)));
    end


    im = imread('testimage.png');
    figure; imshow(im);
    title('Original Image');
    
    % 5x5 gaus filtresiyle görüntüyü yumuþat
    im = rgb2gray(im);
    
    
    
    im = double(imgaussfilt(im,sigma));
    
   
    
    % Resmin yoðunluk derecesini bulun
    Gx = SobelFilter(im, 'x');
    Gy = SobelFilter(im, 'y');
    Gx = imgaussfilt(Gx,sigma);
    Gy = imgaussfilt(Gy,sigma);
    
  
    
    % gradientýn büyüklüðünü bul
    Gmag = sqrt(Gx.^2 + Gy.^2);
    angle = atan2(Gy,Gx)*180/pi;
    
    
         
    % deðiþim olmadan max deðiþim gerçekleþtirir
    [h,w] = size(im);
    X=[-1,0,+1 ;-1,0,+1 ;-1,0,+1];

	Y=[-1,-1,-1 ;0,0,0 ;+1,+1,+1];
    output = zeros(h,w);
    x = [0 1];
    for i=2:h-1 % row
        for j=2:w-1 % col         
            if (angle(i,j)>=-22.5 && angle(i,j)<=22.5) || ...
                (angle(i,j)<-157.5 && angle(i,j)>=-180)
                if (Gmag(i,j) >= Gmag(i,j+1)) && ...
                   (Gmag(i,j) >= Gmag(i,j-1))
                    output(i,j)= Gmag(i,j);
                else
                    output(i,j)=0;
                end
            elseif (angle(i,j)>=22.5 && angle(i,j)<=67.5) || ...
                (angle(i,j)<-112.5 && angle(i,j)>=-157.5)
                if (Gmag(i,j) >= Gmag(i+1,j+1)) && ...
                   (Gmag(i,j) >= Gmag(i-1,j-1))
                    output(i,j)= Gmag(i,j);
                else
                    output(i,j)=0;
                end
            elseif (angle(i,j)>=67.5 && angle(i,j)<=112.5) || ...
                (angle(i,j)<-67.5 && angle(i,j)>=-112.5)
                if (Gmag(i,j) >= Gmag(i+1,j)) && ...
                   (Gmag(i,j) >= Gmag(i-1,j))
                    output(i,j)= Gmag(i,j);
                else
                    output(i,j)=0;
                end
            elseif (angle(i,j)>=112.5 && angle(i,j)<=157.5) || ...

                (angle(i,j)<-22.5 && angle(i,j)>=-67.5)
                if (Gmag(i,j) >= Gmag(i+1,j-1)) && ...
                   (Gmag(i,j) >= Gmag(i-1,j+1))
                    output(i,j)= Gmag(i,j);
                else
                    output(i,j)=0;
                end
            end
        end
    end
    
    Gmag = NormalizeMatrix(output);
    
    
    
    % thresholding yapar
    highThreshold = max(max(Gmag))*highThresholdRatio;
    lowThreshold = highThreshold*lowThresholdRatio;
    strongEdgesRow = zeros(1,h); % satýrdaki en güçlü kenarý takip etme
    strongEdgesCol = zeros(1,w); % sütundaki en güçlü kenarý takip etme
    weakEdgesRow = zeros(1,h);  % satýrdaki en zayýf kenarý takip etme
    weakEdgesCol = zeros(1,w);  % sütundaki en zayýf kenarý takip etme
    strongIndex = 1;
    weakIndex = 1;
    for i=2:h-1 % row
        for j=2:w-1 % col
            if Gmag(i,j) > highThreshold    % güçlü kenar
                Gmag(i,j) = 1;
                strongEdgesRow(strongIndex) = i;
                strongEdgesCol(strongIndex) = j;

                strongIndex = strongIndex + 1;
            elseif Gmag(i,j) < lowThreshold % kenar yok
                Gmag(i,j) = 0;
            else                            % zayýf kenar
                weakEdgesRow(weakIndex) = i;
                weakEdgesCol(weakIndex) = j;
                weakIndex = weakIndex + 1;
            end
        end
    end
     
    
    % Perform edge tracking by hysteresis
    set(0,'RecursionLimit',10000)
    for i=1:strongIndex-1
        % Güclü kenarlara baðlý ve ayarlanmýþ zayýf kenarlarý bulur
        % ve 1 yapar
        Gmag = FindConnectedWeakEdges(Gmag, strongEdgesRow(i),...
            strongEdgesCol(i));
    end
    
   
    
    % zayýf kenarlarý çýkarma ve yerine gürültü koyma
    
    for i=1:weakIndex-1
        if Gmag(weakEdgesRow(i),weakEdgesCol(i)) ~= 1
            Gmag(weakEdgesRow(i),weakEdgesCol(i)) = 0;
        end
    end

    figure; imshow(Gmag);
     
    
   
end

% Normalize matrix
function[A] = NormalizeMatrix(A)
    A = A/max(A(:));
end

% 	sobel filtresi uygulamasý
function[A] = SobelFilter(A, filterDirection)
    switch filterDirection
        case 'x' 
            Gx = [-1 0 +1; -2 0 +2; -1 0 +1];
            A = imfilter(A, double(Gx), 'conv', 'replicate');
        case 'y'
            Gy = [-1 -2 -1; 0 0 0; +1 +2 +1];
            A = imfilter(A, double(Gy), 'conv', 'replicate');
        otherwise
            error('Yanlis filtre girisi - ''x'' veya ''y''');
    end
end

% yüksek deðerleri kenarlara baðlý olan düþük deðerli kenarlarý baðlar
function[Gmag] = FindConnectedWeakEdges(Gmag, row, col)
    for i = -2:1:2
        for j = -2:1:2
            if (row+i > 0) && (col+j > 0) && (row+i < size(Gmag,1)) && ...

                    (col+j < size(Gmag,2)) % resmin sýnýrlarýnýn dýþýna filtre atmýyor
                if (Gmag(row+i,col+j) > 0) && (Gmag(row+i,col+j) < 1)
                    Gmag(row+i,col+j) = 1;
                    Gmag = FindConnectedWeakEdges(Gmag, row+i, col+j);
                end
            end
        end
    end
end
