#include "graph_visualization.hpp"

#include "scene_graph.hpp"

#include <vtkCircularLayoutStrategy.h>
#include <vtkDataSetAttributes.h>
#include <vtkDoubleArray.h>
#include <vtkGraphLayoutView.h>
#include <vtkIntArray.h>
#include <vtkMutableUndirectedGraph.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderedGraphRepresentation.h>
#include <vtkRenderer.h>
#include <vtkTextProperty.h>

auto vtkGraphFromSceneGraph(const SceneGraph& graph) -> vtkNew<vtkMutableUndirectedGraph>
{
    vtkNew<vtkMutableUndirectedGraph> g;
    vtkNew<vtkStringArray> vertex_labels;
    vertex_labels->SetNumberOfComponents(1);
    vertex_labels->SetName("vertex-labels");
    vtkNew<vtkStringArray> edge_labels;
    edge_labels->SetNumberOfComponents(1);
    edge_labels->SetName("edge-labels");
    
    for (const auto o : graph.objects)
    {
        const auto gv = g->AddVertex();
        vertex_labels->InsertNextValue(o.get().description);
        
        for (const auto& f : o.get().features) {
            const auto fv = g->AddVertex();
            vertex_labels->InsertNextValue(f.description);
            
            g->AddEdge(gv, fv);
        }
    }
    
    g->GetVertexData()->AddArray(vertex_labels);
    g->GetEdgeData()->AddArray(edge_labels);
    
    return g;
}

auto view(const SceneGraph& graph) -> void
{
    vtkNew<vtkNamedColors> colors;
    
    const auto g = vtkGraphFromSceneGraph(graph);
    
    vtkNew<vtkCircularLayoutStrategy> circularLayoutStrategy;
    
    vtkNew<vtkGraphLayoutView> graphLayoutView;
    graphLayoutView->AddRepresentationFromInput(g);
    
    graphLayoutView->SetLayoutStrategy(circularLayoutStrategy);
    graphLayoutView->SetVertexLabelVisibility(true);
//    graphLayoutView->SetEdgeLabelVisibility(true);
//    graphLayoutView->SetEdgeLabelArrayName("edge-labels");     // default is "labels"
    graphLayoutView->SetVertexLabelArrayName("vertex-labels"); // default is "labels"
    dynamic_cast<vtkRenderedGraphRepresentation*>(
                                                  graphLayoutView->GetRepresentation())
    ->GetVertexLabelTextProperty()
    ->SetColor(colors->GetColor3d("Yellow").GetData());
    dynamic_cast<vtkRenderedGraphRepresentation*>(
                                                  graphLayoutView->GetRepresentation())
    ->GetEdgeLabelTextProperty()
    ->SetColor(colors->GetColor3d("Lime").GetData());
    graphLayoutView->GetRenderer()->SetBackground(
                                                  colors->GetColor3d("Navy").GetData());
    graphLayoutView->GetRenderer()->SetBackground2(
                                                   colors->GetColor3d("MidnightBlue").GetData());
    graphLayoutView->GetRenderWindow()->SetWindowName("LabelVerticesAndEdges");
    graphLayoutView->ResetCamera();
    graphLayoutView->Render();
    graphLayoutView->GetInteractor()->Start();
}
