#!/bin/bash
# Isaac Lab Experiment Dashboard Launcher
# 
# This script sets up and launches the interactive experiment dashboard

echo "🤖 Isaac Lab Experiment Dashboard Launcher"
echo "=========================================="

# Check if streamlit is installed
if ! command -v streamlit &> /dev/null; then
    echo "❌ Streamlit not found. Installing requirements..."
    pip install -r dashboard_requirements.txt
    
    if [ $? -ne 0 ]; then
        echo "❌ Failed to install requirements. Please install manually:"
        echo "   pip install -r dashboard_requirements.txt"
        exit 1
    fi
    echo "✅ Requirements installed successfully"
else
    echo "✅ Streamlit found"
fi

# Get the directory of this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo ""
echo "🚀 Starting dashboard..."
echo "📍 Dashboard will be available at: http://localhost:8501"
echo ""
echo "📋 Available features:"
echo "   • Experiment overview and browsing"  
echo "   • Learning curve analysis"
echo "   • Multi-experiment comparisons"
echo "   • Dataset distribution analysis"
echo "   • TensorBoard log visualization"
echo "   • Benchmark result analysis"
echo ""
echo "🛑 To stop the dashboard, press Ctrl+C"
echo ""

# Launch streamlit
cd "$SCRIPT_DIR"
streamlit run experiment_dashboard.py --server.port 8501 --server.address localhost

echo ""
echo "👋 Dashboard stopped."