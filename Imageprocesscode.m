function cannyedgedet ()
    close all;  % hepsini kapatma
    sigma = 1.4; % Gaussian filter sigma
    highThresholdRatio = 0.2; % en y�ksek e�ik oran�
    lowThresholdRatio = 0.15; % en d���k e�ik oran�
    
    % ge�erli dosyay� m uzant�l� file yapmak i�in. �ok gerek yok
    
    if(~isdeployed)
      cd(fileparts(which(mfilename)));
    end


    im = imread('testimage.png');
    figure; imshow(im);
    title('Original Image');
    
    % 5x5 gaus filtresiyle g�r�nt�y� yumu�at
    im = rgb2gray(im);
    
    
    
    im = double(imgaussfilt(im,sigma));
    
   
    
    % Resmin yo�unluk derecesini bulun
    Gx = SobelFilter(im, 'x');
    Gy = SobelFilter(im, 'y');
    Gx = imgaussfilt(Gx,sigma);
    Gy = imgaussfilt(Gy,sigma);
    
  
    
    % gradient�n b�y�kl���n� bul
    Gmag = sqrt(Gx.^2 + Gy.^2);
    angle = atan2(Gy,Gx)*180/pi;
    
    
         
    % de�i�im olmadan max de�i�im ger�ekle�tirir
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
    strongEdgesRow = zeros(1,h); % sat�rdaki en g��l� kenar� takip etme
    strongEdgesCol = zeros(1,w); % s�tundaki en g��l� kenar� takip etme
    weakEdgesRow = zeros(1,h);  % sat�rdaki en zay�f kenar� takip etme
    weakEdgesCol = zeros(1,w);  % s�tundaki en zay�f kenar� takip etme
    strongIndex = 1;
    weakIndex = 1;
    for i=2:h-1 % row
        for j=2:w-1 % col
            if Gmag(i,j) > highThreshold    % g��l� kenar
                Gmag(i,j) = 1;
                strongEdgesRow(strongIndex) = i;
                strongEdgesCol(strongIndex) = j;

                strongIndex = strongIndex + 1;
            elseif Gmag(i,j) < lowThreshold % kenar yok
                Gmag(i,j) = 0;
            else                            % zay�f kenar
                weakEdgesRow(weakIndex) = i;
                weakEdgesCol(weakIndex) = j;
                weakIndex = weakIndex + 1;
            end
        end
    end
     
    
    % Perform edge tracking by hysteresis
    set(0,'RecursionLimit',10000)
    for i=1:strongIndex-1
        % G�cl� kenarlara ba�l� ve ayarlanm�� zay�f kenarlar� bulur
        % ve 1 yapar
        Gmag = FindConnectedWeakEdges(Gmag, strongEdgesRow(i),...
            strongEdgesCol(i));
    end
    
   
    
    % zay�f kenarlar� ��karma ve yerine g�r�lt� koyma
    
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

% 	sobel filtresi uygulamas�
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

% y�ksek de�erleri kenarlara ba�l� olan d���k de�erli kenarlar� ba�lar
function[Gmag] = FindConnectedWeakEdges(Gmag, row, col)
    for i = -2:1:2
        for j = -2:1:2
            if (row+i > 0) && (col+j > 0) && (row+i < size(Gmag,1)) && ...

                    (col+j < size(Gmag,2)) % resmin s�n�rlar�n�n d���na filtre atm�yor
                if (Gmag(row+i,col+j) > 0) && (Gmag(row+i,col+j) < 1)
                    Gmag(row+i,col+j) = 1;
                    Gmag = FindConnectedWeakEdges(Gmag, row+i, col+j);
                end
            end
        end
    end
end
