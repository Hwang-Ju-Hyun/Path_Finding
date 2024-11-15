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

bool Movement::ComputePath( int r, int c, bool newRequest )
{

	m_goal = g_terrain.GetCoordinates( r, c );
	m_movementMode = MOVEMENT_WAYPOINT_LIST;

	//Randomly meander toward goal (might get stuck at wall)
	int curR, curC;
	D3DXVECTOR3 cur = m_owner->GetBody().GetPos();
	g_terrain.GetRowColumn(&cur, &curR, &curC);	

	m_waypointList.clear();
	m_waypointList.push_back(cur);

	// project 2: change this flag to true
	bool useAStar = true;
	int col_cnt = 0;	

	if( useAStar )
	{		
		std::priority_queue<Node,std::vector<Node>,my_comp> open_list;
		std::vector<Node> close_list;
		std::vector<Node> blue_list;
		std::vector<std::pair<int, int>> result_path;
		std::vector<Node> rubber_path;
		int col_cnt = 0;

		int start_Row, start_Col;
		D3DXVECTOR3 cur = m_owner->GetBody().GetPos();
		g_terrain.GetRowColumn(&cur, &start_Row, &start_Col);		

		int target_Row, target_Col;
		D3DXVECTOR3 target = m_goal;
		g_terrain.GetRowColumn(&target, &target_Row, &target_Col);

		int max_RowCol = g_terrain.GetWidth();
		bool close_grid[1000][1000] = { false, };

		Node start_Node;
		start_Node.row = start_Row;
		start_Node.col = start_Col;
		start_Node.gx = 0;
		start_Node.parent = { start_Node.row,start_Node.col };
		start_Node.hx = (abs(target_Row - start_Row) + abs(target_Col - start_Col)) * 10;
		start_Node.fx = start_Node.gx+start_Node.hx;
		
		close_grid[start_Row][start_Col] = true;
		open_list.push(start_Node);
		blue_list.push_back(start_Node);
		

		if (m_straightline)
		{				
			D3DXVECTOR3 spot = { target.x,target.y,target.z };			
			int target_row = g_terrain.GetCoordinates(spot.x, spot.y).x;
			int target_col = g_terrain.GetCoordinates(spot.x, spot.y).y;
			int player_row = g_terrain.GetCoordinates(cur.x, cur.y).x;
			int player_col = g_terrain.GetCoordinates(cur.x, cur.y).y;
			
			m_waypointList.push_back(spot);			
			return true;
		}


		while (!open_list.empty())
		{		
			Node add_node;
								// ↑   →    ↓    ←   ↗  ↘   ↙  ↖ 
			int direct_Row[8] = {  1,  0,  -1,   0,  1,  -1,  -1, 1 };
			int direct_Col[8] = {  0,  1,   0,  -1,  1,  1,  -1, -1 };		
		
			int cur_Row = open_list.top().row;
			int cur_Col = open_list.top().col;									
			int cur_gx = open_list.top().gx;
			close_list.push_back(open_list.top());			
			open_list.pop();

			if (cur_Row == r && cur_Col == c)//찾았당
			{											

				//parent path
				int parent_row = close_list[close_list.size()-1].row;
				int parent_col = close_list[close_list.size()-1].col;
				for (int i = close_list.size()-1; i > 0; i--)
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
									
				DrawPath(blue_list,DEBUG_COLOR_BLUE);
				DrawPath(close_list,DEBUG_COLOR_YELLOW);				
				DrawParentPathAndMove(result_path);
				result_path.clear();
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
				if(g_terrain.IsWall(next_Row,next_Col))
					continue;
				if (close_grid[next_Row][next_Col])
					continue;								

				//대각선으로 벽이 있을 경우
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
				add_node.hx = (abs(target_Row - next_Row) + abs(target_Col - next_Col)) * 10;//맨헤튼				
				add_node.fx = add_node.gx + add_node.hx;
				
				add_node.parent = { cur_Row,cur_Col };
				close_grid[next_Row][next_Col] = true;
				blue_list.push_back(add_node);
				open_list.push(add_node); 				
			}
		}
		return false;
	}
	else
	{	
		//Randomly meander toward goal (might get stuck at wall)
		int curR, curC;
		D3DXVECTOR3 cur = m_owner->GetBody().GetPos();
		g_terrain.GetRowColumn( &cur, &curR, &curC );

		m_waypointList.clear();
		m_waypointList.push_back( cur );

		int countdown = 100;
		while( curR != r || curC != c )
		{
			if( countdown-- < 0 ) { break; }

			if( curC == c || (curR != r && rand()%2 == 0) )
			{	//Go in row direction
				int last = curR;
				if( curR < r ) { curR++; }
				else { curR--; }

				if( g_terrain.IsWall( curR, curC ) )
				{
					curR = last;
					continue;
				}
			}
			else
			{	//Go in column direction
				int last = curC;
				if( curC < c ) { curC++; }
				else { curC--; }

				if( g_terrain.IsWall( curR, curC ) )
				{
					curC = last;
					continue;
				}
			}

			D3DXVECTOR3 spot = g_terrain.GetCoordinates( curR, curC );
			m_waypointList.push_back( spot );
			g_terrain.SetColor( curR, curC, DEBUG_COLOR_YELLOW );
		}
		return true;
	}
}
