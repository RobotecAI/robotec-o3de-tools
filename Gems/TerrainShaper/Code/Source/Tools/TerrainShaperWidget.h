
#pragma once

#if !defined(Q_MOC_RUN)
#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <QWidget>
#endif

namespace TerrainShaper
{
    class TerrainShaperWidget
        : public QWidget
    {
        Q_OBJECT
    public:
        explicit TerrainShaperWidget(QWidget* parent = nullptr);
    };
} 
