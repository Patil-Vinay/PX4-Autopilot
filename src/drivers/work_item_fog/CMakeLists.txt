px4_add_module(
	MODULE drivers__work_item_fog
	MAIN work_item_fog
	COMPILE_FLAGS
		#-Wno-cast-align
	SRCS
		WorkItemFOG.cpp
		WorkItemFOG.hpp
	DEPENDS
		px4_work_queue
	)
