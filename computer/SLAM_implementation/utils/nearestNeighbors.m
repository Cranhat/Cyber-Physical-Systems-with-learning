function idx = nearestNeighbors(points, reference)
    idx = zeros(size(points, 1), 1);

    for i = 1:size(points, 1)
        diff = reference - points(i, :);
        dist2 = sum(diff.^2, 2);
        [~, idx(i)] = min(dist2);
    end
end