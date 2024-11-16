/* Copyright Steve Rabin, 2012. 
 * All rights reserved worldwide.
 *
 * This software is provided "as is" without express or implied
 * warranties. You may freely copy and compile this source into
 * applications you distribute provided that the copyright text
 * below is included in the resulting source code, for example:
 * "Portions Copyright Steve Rabin, 2012"
 */

#include <Stdafx.h>
#include <queue>

typedef struct node
{
public:
	int row, col;
	int gx, fx;
	int hx;
	std::pair<int, int> parent;
}Node;

struct my_comp
{
	bool operator()(Node a, Node b)
	{
		if (a.fx > b.fx)
			return true;
		else if (a.fx == b.fx)
		{
			if (a.gx < b.gx)
				return true;
			else
				return false;
		}
		return false;
	}
};

enum HEURISTIC
{
    MANHATTAN=0,
    OTILE=1,
    CHEVYSHEV=2,    
    EUCLIDEAN=3
};

int GetHeuristic(int _startRow,int _startCol,int _targetRow, int _targetCol, enum HEURISTIC _h)
{
    if (_h == HEURISTIC::MANHATTAN)
    {
        return (abs(_targetRow - _startRow) + abs(_targetCol - _startCol)) * 10;
    }
    else if (_h == HEURISTIC::OTILE)
    {
        return std::sqrt(std::pow(_targetRow - _startRow, 2) + (std::pow(_targetCol - _startCol, 2), 2));
    }    
}

void DrawPath(const std::vector<Node>& _path, DebugDrawingColor _eColor)
{
	for (int i = 0; i < _path.size() - 1; i++)
	{
		g_terrain.SetColor(_path[i].row, _path[i].col, _eColor);
	}
}

void Movement::DrawParentPathAndMove(const std::vector<std::pair<int, int>>& _path)
{
	for (int i = _path.size() - 1; i > 0; i--)
	{
		D3DXVECTOR3 spot = g_terrain.GetCoordinates(_path[i].first, _path[i].second);
		m_waypointList.push_back(spot);
		g_terrain.SetColor(_path[i].first, _path[i].second, DEBUG_COLOR_YELLOW);
	}	
}

bool IsLineClear(int startRow, int startCol, int endRow, int endCol)
{
    // Bresenham's Line Algorithm or similar technique to check for obstacles in a straight line
    int delta_col = abs(endCol - startCol);
    int delta_row = abs(endRow - startRow);
    int col = (startCol < endCol) ? 1 : -1;
    int row = (startRow < endRow) ? 1 : -1;
    int err = delta_col - delta_row;

    // Determine the number of steps required based on the largest dimension (dx or dy)
    int maxSteps = max(delta_col, delta_row);

    // Use a for loop to iterate through each step from the start to the end
    for (int i = 0; i <= maxSteps; i++)
    {
        // If there's a wall at the current point, return false
        if (g_terrain.IsWall(startRow, startCol))
            return false;

        // If we've reached the end point, break out of the loop
        if (startRow == endRow && startCol == endCol)
            break;

        // Update the error term and adjust the row/column as necessary
        int e2 = err * 2;

        if (e2 > -delta_row)
        {
            err -= delta_row;
            startCol += col;
        }
        if (e2 < delta_col)
        {
            err += delta_col;
            startRow += row;
        }
    }
    return true;
}

void erase_element(std::vector<std::pair<int, int>>* _vec,int _idx)
{        
    for (int i = 0; i < _vec->size();)
    {
        if (i == _idx)
        {
            _vec->erase(_vec->begin() + i);
            break;
        }
        else
        {
            i++;
        }            
    }
}


bool Movement::ComputePath(int r, int c, bool newRequest)
{
    m_goal = g_terrain.GetCoordinates(r, c);
    m_movementMode = MOVEMENT_WAYPOINT_LIST;

    int curR, curC;
    D3DXVECTOR3 cur = m_owner->GetBody().GetPos();
    g_terrain.GetRowColumn(&cur, &curR, &curC);

    m_waypointList.clear();
    m_waypointList.push_back(cur);

    bool useAStar = true;
    int col_cnt = 0;

    if (useAStar)
    {
        std::priority_queue<Node, std::vector<Node>, my_comp> open_list;
        std::vector<Node> close_list;
        std::vector<Node> blue_list;
        std::vector<std::pair<int, int>> result_path;
        std::vector<Node> rubber_path;
        int col_cnt = 0;

        int start_Row, start_Col;
        int cur_Row, cur_Col;
        D3DXVECTOR3 cur = m_owner->GetBody().GetPos();
        g_terrain.GetRowColumn(&cur, &start_Row, &start_Col);
        g_terrain.GetRowColumn(&cur, &cur_Row, &cur_Col);

        int target_Row, target_Col;
        D3DXVECTOR3 target = m_goal;
        g_terrain.GetRowColumn(&target, &target_Row, &target_Col);

        int max_RowCol = g_terrain.GetWidth();
        bool close_grid[1000][1000] = { false, };

        Node start_Node;
        start_Node.row = start_Row;
        start_Node.col = start_Col;
        start_Node.gx = 0;
        start_Node.parent = { start_Node.row, start_Node.col };
        start_Node.hx = 0;//GetHeuristic(start_Row,start_Col,target_Row,target_Col,HEURISTIC::MANHATTAN);
        start_Node.fx = start_Node.gx + start_Node.hx;

        close_grid[start_Row][start_Col] = true;
        open_list.push(start_Node);
        blue_list.push_back(start_Node);
        
        if (m_straightline)
        {

            if (IsLineClear(cur_Row, cur_Col, target_Row, target_Col))
            {
                D3DXVECTOR3 spot = { target.x, target.y, target.z };
                m_waypointList.push_back(spot);
                return true;
            }
        }        

        while (!open_list.empty())
        {
            Node add_node;
            int direct_Row[8] = { 1, 0, -1, 0, 1, -1, -1, 1 };
            int direct_Col[8] = { 0, 1, 0, -1, 1, 1, -1, -1 };

            int cur_Row = open_list.top().row;
            int cur_Col = open_list.top().col;
            int cur_gx = open_list.top().gx;
            close_list.push_back(open_list.top());
            open_list.pop();

            if (cur_Row == r && cur_Col == c)
            {
                int parent_row = close_list[close_list.size() - 1].row;
                int parent_col = close_list[close_list.size() - 1].col;
                for (int i = close_list.size() - 1; i > 0; i--)
                {
                    if (parent_row == close_list[i].row && parent_col == close_list[i].col)
                    {
                        parent_row = close_list[i].parent.first;
                        parent_col = close_list[i].parent.second;
                        if (parent_row == start_Node.row && parent_col == start_Node.col)
                        {
                            result_path.push_back({ parent_row, parent_col });
                            break;
                        }
                        result_path.push_back({ parent_row, parent_col });
                    }   
                }       
                std::vector<std::pair<int, int>> rubberband_path;
                if (m_rubberband)
                {                    
                    for(int i = 0; i < result_path.size(); i++)                    
                        rubberband_path.push_back({ result_path[i].first,result_path[i].second });
                        
                    for (int i = rubberband_path.size()-1; i >0; i--)
                    {    
                        if (i <= 2)
                        {                            
                            break;
                        }
                        int n1_row = rubberband_path[i].first;
                        int n1_col = rubberband_path[i].second;
                        int n2_row = rubberband_path[i-1].first;
                        int n2_col = rubberband_path[i - 1].second;
                        int n3_row = rubberband_path[i - 2].first;
                        int n3_col = rubberband_path[i - 2].second;
                        if (IsLineClear(n1_row,n1_col,n3_row,n3_col))
                        {
                            erase_element(&rubberband_path, i-1);
                        }
                        else
                        {
                            continue;
                        }                        
                    }
                }
                if (m_rubberband)
                {
                    DrawParentPathAndMove(rubberband_path);                    
                }
                else
                {
                    DrawPath(blue_list, DEBUG_COLOR_BLUE);
                    DrawPath(close_list, DEBUG_COLOR_YELLOW);
                    DrawParentPathAndMove(result_path);
                    result_path.clear();
                }                
                return true;
            }

            for (int i = 0; i < 8; i++)
            {
                int next_Row = cur_Row + direct_Row[i];
                int next_Col = cur_Col + direct_Col[i];

                if (next_Row < 0 || next_Col < 0)
                    continue;
                if (next_Row >= max_RowCol || next_Col >= max_RowCol)
                    continue;
                if (g_terrain.IsWall(next_Row, next_Col))
                    continue;
                if (close_grid[next_Row][next_Col])
                    continue;

                if (i >= 4)
                {
                    int adj_Row1 = cur_Row + direct_Row[i];
                    int adj_Col1 = cur_Col;
                    int adj_Row2 = cur_Row;
                    int adj_Col2 = cur_Col + direct_Col[i];
                    if (g_terrain.IsWall(adj_Row1, adj_Col1) || g_terrain.IsWall(adj_Row2, adj_Col2))
                        continue;
                }

                add_node.row = next_Row;
                add_node.col = next_Col;

                if (i < 4)
                {
                    add_node.gx = cur_gx + 10;
                }
                else
                {
                    add_node.gx = cur_gx + 14;
                }
                add_node.hx = 0;//(abs(target_Row - next_Row) + abs(target_Col - next_Col)) * 10;
                add_node.fx = add_node.gx + add_node.hx;

                add_node.parent = { cur_Row, cur_Col };
                close_grid[next_Row][next_Col] = true;
                blue_list.push_back(add_node);
                open_list.push(add_node);
            }
        }
        return false;
    }
    else
    {
        int curR, curC;
        D3DXVECTOR3 cur = m_owner->GetBody().GetPos();
        g_terrain.GetRowColumn(&cur, &curR, &curC);

        m_waypointList.clear();
        m_waypointList.push_back(cur);

        int countdown = 100;
        while (curR != r || curC != c)
        {
            if (countdown-- < 0) { break; }

            if (curC == c || (curR != r && rand() % 2 == 0))
            {
                int last = curR;
                if (curR < r) { curR++; }
                else { curR--; }

                if (g_terrain.IsWall(curR, curC))
                {
                    curR = last;
                    continue;
                }
            }
            else
            {
                int last = curC;
                if (curC < c) { curC++; }
                else { curC--; }

                if (g_terrain.IsWall(curR, curC))
                {
                    curC = last;
                    continue;
                }
            }

            D3DXVECTOR3 spot = g_terrain.GetCoordinates(curR, curC);
            m_waypointList.push_back(spot);
            g_terrain.SetColor(curR, curC, DEBUG_COLOR_YELLOW);
        }
        return true;
    }
}
