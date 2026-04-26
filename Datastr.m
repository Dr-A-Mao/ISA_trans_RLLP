%% 可随意增加、删除和索引的类
classdef Datastr < handle
    properties
        %% 状态
        data
        length % 序列长度
    end
    methods
        %% 初始化函数
        function obj = Datastr()
            obj.data = {};
            obj.length = 0;
        end
        %% 在尾部追加结点
        function append_elements(obj,value)
            data_ = obj.data;
            obj.data = [data_,value];
            obj.length = obj.length + 1;
        end
        %% 在索引k处插入元素
        function insert_elements(obj,k,value)
            % k <= length
            data_ = obj.data;
            if obj.length == 0
                obj.data = {value};
            elseif obj.length == 1
                obj.data = [data_(1),value];
            else
                if k == obj.length
                   obj.data = [data_,value];
                else
                    obj.data = [data_(1:k),value,data_(k+1:end)];
                end
            end
            obj.length = obj.length + 1;
        end
        %% 在索引k处删除元素
        function delete_elements(obj,k)
            data_ = obj.data;
            if obj.length == 0
                disp('报错，无法删除元素！');
                return;
            elseif obj.length == 1
                obj.data = {};
            elseif obj.length == 2
                if k == 1
                    obj.data = {data_{2}};
                else
                    obj.data = {data_{1}};
                end
            else
                if k == obj.length
                    obj.data = data_(1:k-1);
                elseif k == 1 
                    obj.data = data_(2:end);
                else
                    obj.data = [data_(1:k-1),data_(k+1:end)];
                end
            end
            obj.length = obj.length - 1;
        end
        %% 在索引k处替换元素
        function modify_elements(obj,k,value)
            obj.data{k} = value;
        end
        %% 克隆方法，实现数据结构的不关联传递
        function new_object = clone(obj)
            new_object = Datastr();
            new_object.data = obj.data;
            new_object.length = obj.length;
        end
    end
end