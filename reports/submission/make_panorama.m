function [panorama] = make_panorama(images_dir)

    % read panorama source images
    files = dir(strcat(images_dir, '*.png'));
    
    % colour correction factor
    gamma = 2.2;
    alpha = ones(1, length(files));
    for i=2:length(files)
        I = imread(strcat(images_dir, files(i).name));
        J = imread(strcat(images_dir, files(i-1).name));
        alpha(i) = sum(double(J(:, :)).^gamma)/sum(double(I(:, :)).^gamma);
        
    end
    
    % overall color normalizing factor
    G = sum(alpha, 2)/sum(alpha.^2, 2);
    
    % camera shift for each picture
    shift = 125;
    
    % construct panorama
    panorama = imread(strcat(images_dir, files(1).name));
    for i=2:length(files)
        curr_img = double(imread(strcat(images_dir, files(i).name))); 
 
        % colour correction factor
        factor = alpha(i)*G;
        curr_img(:, :) = curr_img(:, :)*factor;
        
        % error surface
        overlap = size(curr_img, 2)-shift;
        e = (double(panorama(:, end-overlap:end))-double(curr_img(:, 1:overlap+1))).^2;
       
        % optimal seam finding
        E = zeros(size(e));
        E(1, :) = e(1, :);
        for h=2:size(e, 1)
            for w=1:size(e, 2)
                if w == 1
                    cost = min([E(h-1, w), E(h-1, w+1)]);
                elseif w == size(e, 2)
                    cost = min([E(h-1, w-1), E(h-1, w)]);
                else
                    cost = min([E(h-1, w-1), E(h-1, w), E(h-1, w+1)]);
                end
                E(h, w) = e(h, w) + cost;
            end
        end

        % traceback for optimal path
        h = size(e, 1);
        path = zeros(h, 1);
        [val, idx] = min(E(h, :));
        path(h) = idx;
        for h=size(e, 1)-1:-1:1
            w = path(h+1);
            if w > 1 && E(h, w-1) == E(h+1, w)-e(h+1, w)
                path(h) = w-1;
            elseif w < size(e, 2) && E(h, w+1) == E(h+1, w)-e(h+1, w)
                path(h) = w+1;
            else
                path(h) = w;
            end
        end
        
        % sticth the path over seam
        tmp = [];
        bound_threshold = 30;
        for h=1:size(panorama, 1)
            A = panorama(h, 1:end-(overlap-path(h)+1));
            B = curr_img(h, path(h):end);
            Z = cat(2, A, B);
            filt_A = ones(1, size(A, 2)-bound_threshold);
            grad = 1:-1/(2*bound_threshold):0;
            filt_B = zeros(1, size(B, 2)-bound_threshold);
            blender = cat(2, filt_A, grad, filt_B);
            Z = blender(:, 1:size(Z, 2)).*double(Z) + (1-blender(:, 1:size(Z, 2))).*double(Z);
            tmp = [tmp; Z;];
        end
        panorama = tmp;
        
    end
    
    % show panorama
    imshow(uint8(panorama))
    imwrite(uint8(panorama), './results/robot-7.png');
end