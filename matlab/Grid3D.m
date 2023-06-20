classdef Grid3D < handle
    properties
        origin;
        resolution;
        width;
        height;
        depth;

        log_odds_hit; log_odds_miss;
        min_clamp; max_clamp;
        track_changes;
        occupancy_threshold;
        free_threshold;
        block_size;
	lock_at_max_clamp;
    end
    properties (SetAccess = private, Hidden = true)
        objectHandle;
    end
    methods (Access = private)
        function setObjectHandle(this, obj_handle)
            this.objectHandle = obj_handle;
        end
    end

    methods
        function this = Grid3D(in)
        % nargin can also be zero
	  if (isa(in, 'char') || isa(in, 'string'))
	    this.objectHandle = grid3d_mex('new', in);
	    this.getParameters();
	  else
            this.objectHandle = in;
	  end
        end
        function printParameters(this)
            grid3d_mex('print_parameters', this.objectHandle);
        end
        function [xyz, probs] = getXYZProbability(this)
            [xyz, probs] = grid3d_mex('get_xyz_probability', this.objectHandle);
        end
        function p = probability(this, pt)
            p = grid3d_mex('probability', this.objectHandle, pt);
        end
        function p = logodds(this, idx)
            p = grid3d_mex('logodds', this.objectHandle, idx);
        end
        function idxs = getIndex(this, pts)
            in = pts;
            if (size(in,1) ~= 3)
                in = transpose(pts);
            end
            if (size(in,1) ~= 3)
                error('size of pts should be 3xN');
            end

            idxs = grid3d_mex('get_index', this.objectHandle(), in);
        end
        function [] = getParameters(this)
            [this.origin, this.resolution, this.width, this.height, this.depth,...
            this.log_odds_hit, this.log_odds_miss, this.min_clamp, this.max_clamp, ...
            this.track_changes, this.occupancy_threshold, this.free_threshold, ...
            this.block_size, this.lock_at_max_clamp] = grid3d_mex('get_parameters', this.objectHandle());
        end
        function probabilities = getProbability(this)
            probabilities = grid3d_mex('get_probability', this.getObjectHandle());
        end
        function [objectHandle] = getObjectHandle(this)
            objectHandle = this.objectHandle;
        end
        function [pts] = getPoint(this, idxs)
            pts = grid3d_mex('get_point', this.objectHandle(), idxs);
        end
        function [pts, ret] = getRayPoints(this, st, en)
            [pts, ret] = grid3d_mex('get_ray_points', this.objectHandle(), st, en);
        end
	function ret = inQCell(this, c)
	  ret = grid3d_mex('in_q_cell', this.objectHandle(), c);
	end
	function ret = inQPoint(this, pt)
	  ret = grid3d_mex('in_q_point', this.objectHandle(), pt);
	end
	function out = w2c(this, pt)
	  out = grid3d_mex('w2c', this.objectHandle(), pt);
	end
	function out = getNumChanges(this)
	  out = grid3d_mex('get_num_changes', this.objectHandle());
	end
	function [] = resetChangeSet(this)
	  grid3d_mex('reset_change_set', this.objectHandle());
	end
	function [] = addRay(this, st, en, max_range)
	  grid3d_mex('add_ray', this.objectHandle(), st, en, max_range);
	end
	function [] = plot(this, color, alpha)
	  if isempty(this.origin)
	    this.getParameters();
	  end
	  bbx = BoundingBox();
	  bbx.min = this.origin;
	  bbx.max = this.origin + this.resolution*[this.width; this.height; this.depth];
	  bbx.plot(color, alpha);
	end
        function delete(this)
            if (this.objectHandle ~= 0)
                grid3d_mex('delete', this.objectHandle);
            end
        end
    end
end
