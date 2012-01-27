// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// 
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "hyper_graph_action.h"
#include "optimizable_graph.h"
#include "g2o/stuff/macros.h"


#include <iostream>

namespace g2o {
  using namespace std;

  HyperGraphActionLibrary* HyperGraphActionLibrary::actionLibInstance = 0;

  HyperGraphAction::Parameters::~Parameters()
  {
  }

  HyperGraphAction::ParametersIteration::ParametersIteration(int iter) :
    HyperGraphAction::Parameters(),
    iteration(iter)
  {
  }

  HyperGraphAction::~HyperGraphAction()
  {
  }

  HyperGraphAction* HyperGraphAction::operator()(const HyperGraph*, Parameters*)
  {
    return 0;
  }

  HyperGraphElementAction::Parameters::~Parameters()
  {
  }

  HyperGraphElementAction::HyperGraphElementAction(const std::string& typeName_)
  {
    _typeName = typeName_;
  }

  void HyperGraphElementAction::setTypeName(const std::string& typeName_)
  {
    _typeName = typeName_;
  }


  HyperGraphElementAction* HyperGraphElementAction::operator()(HyperGraph::HyperGraphElement* , HyperGraphElementAction::Parameters* )
  {
    return 0;
  }
  
  HyperGraphElementAction* HyperGraphElementAction::operator()(const HyperGraph::HyperGraphElement* , HyperGraphElementAction::Parameters* )
  {
    return 0;
  }
  
  HyperGraphElementAction::~HyperGraphElementAction()
  {
  }

  HyperGraphElementActionCollection::HyperGraphElementActionCollection(const std::string& name_)
  {
    _name = name_;
  }

  HyperGraphElementActionCollection::~HyperGraphElementActionCollection()
  {
    for (ActionMap::iterator it = _actionMap.begin(); it != _actionMap.end(); ++it) {
      delete it->second;
    }
  }

  HyperGraphElementAction* HyperGraphElementActionCollection::operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params)
  {
    ActionMap::iterator it=_actionMap.find(typeid(*element).name());
    //cerr << typeid(*element).name() << endl;
    if (it==_actionMap.end())
      return 0;
    HyperGraphElementAction* action=it->second;
    return (*action)(element, params);
  }

  HyperGraphElementAction* HyperGraphElementActionCollection::operator()(const HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params)
  {
    ActionMap::iterator it=_actionMap.find(typeid(*element).name());
    if (it==_actionMap.end())
      return 0;
    HyperGraphElementAction* action=it->second;
    return (*action)(element, params);
  }

  bool HyperGraphElementActionCollection::registerAction(HyperGraphElementAction* action)
  {
    if (action->name()!=name()){
      cerr << __PRETTY_FUNCTION__  << ": invalid attempt to register an action in a collection with a different name " <<  name() << " " << action->name() << endl;
    }
    _actionMap.insert(make_pair ( action->typeName(), action) );
    return true;
  }

  bool HyperGraphElementActionCollection::unregisterAction(HyperGraphElementAction* action)
  {
    for (HyperGraphElementAction::ActionMap::iterator it=_actionMap.begin(); it != _actionMap.end(); ++it)
      {
        if (it->second == action)
          {
            _actionMap.erase(it);
            return true;
          }
      }
    return false;
  }
  
  HyperGraphActionLibrary::HyperGraphActionLibrary()
  {
  }

  HyperGraphActionLibrary* HyperGraphActionLibrary::instance()
  {
    if (actionLibInstance == 0) {
      actionLibInstance = new HyperGraphActionLibrary;
    }
    return actionLibInstance;
  }

  void HyperGraphActionLibrary::destroy()
  {
    delete actionLibInstance;
    actionLibInstance = 0;
  }

  HyperGraphActionLibrary::~HyperGraphActionLibrary()
  {
    for (HyperGraphElementAction::ActionMap::iterator it = _actionMap.begin(); it != _actionMap.end(); ++it) {
      delete it->second;
    }
  }
  
  HyperGraphElementAction* HyperGraphActionLibrary::actionByName(const std::string& name)
  {
    HyperGraphElementAction::ActionMap::iterator it=_actionMap.find(name);
    if (it!=_actionMap.end())
      return it->second;
    return 0;
  }

  bool HyperGraphActionLibrary::registerAction(HyperGraphElementAction* action)
  {
    HyperGraphElementAction* oldAction = actionByName(action->name());
    HyperGraphElementActionCollection* collection = 0;
    if (oldAction) {
      collection = dynamic_cast<HyperGraphElementActionCollection*>(oldAction);
      if (! collection) {
        cerr << __PRETTY_FUNCTION__ << ": fatal error, a collection is not at the first level in the library" << endl;
        return 0;
      }
    }
    if (! collection) {
      cerr << __PRETTY_FUNCTION__ << ": creating collection for \"" << action->name() << "\"" << endl;
      collection = new HyperGraphElementActionCollection(action->name());
      _actionMap.insert(make_pair(action->name(), collection));
    }
    return collection->registerAction(action);
  }
  
  bool HyperGraphActionLibrary::unregisterAction(HyperGraphElementAction* action)
  {
    
    list<HyperGraphElementActionCollection*> collectionDeleteList;

    // Search all the collections and delete the registered actions; if a collection becomes empty, schedule it for deletion; note that we can't delete the collections as we go because this will screw up the state of the iterators
    for (HyperGraphElementAction::ActionMap::iterator it=_actionMap.begin(); it != _actionMap.end(); ++it)
      {
        HyperGraphElementActionCollection* collection = dynamic_cast<HyperGraphElementActionCollection*> (it->second);
        if (collection != 0)
          {
            collection->unregisterAction(action);
            if (collection->actionMap().size() == 0)
              {
                collectionDeleteList.push_back(collection);
              }
          }
      }

    // Delete any empty action collections
    for (list<HyperGraphElementActionCollection*>::iterator itc = collectionDeleteList.begin();
         itc != collectionDeleteList.end(); ++itc)
      {
        cout << "Deleting collection " << (*itc)->name() << endl;
        _actionMap.erase((*itc)->name());
      }

    return true;
  }


  WriteGnuplotAction::WriteGnuplotAction(const std::string& typeName_)
    : HyperGraphElementAction(typeName_)
  {
    _name="writeGnuplot";
  }

  DrawAction::Parameters::Parameters(){
  }

  DrawAction::DrawAction(const std::string& typeName_) 
    : HyperGraphElementAction(typeName_)
  {
    _name="draw";
    _previousParams = (Parameters*)0x42;
    refreshPropertyPtrs(0);
  }

  bool DrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_){
    if (_previousParams == params_)
      return false;
    DrawAction::Parameters* p=dynamic_cast<DrawAction::Parameters*>(params_);
    if (! p){
      _previousParams = 0;
      _show = 0;
      _showId = 0;
    } else {
      _previousParams = p;
      _show = p->makeProperty<BoolProperty>(_typeName+"::SHOW", true);
      _showId = p->makeProperty<BoolProperty>(_typeName+"::SHOW_ID", false);
    }
    return true;
  }

  void applyAction(HyperGraph* graph, HyperGraphElementAction* action, HyperGraphElementAction::Parameters* params, const std::string& typeName)
  {
    for (HyperGraph::VertexIDMap::iterator it=graph->vertices().begin(); 
        it!=graph->vertices().end(); ++it){
      if ( typeName.empty() || typeid(*it->second).name()==typeName){
        (*action)(it->second, params);
      }
    }
    for (HyperGraph::EdgeSet::iterator it=graph->edges().begin(); 
        it!=graph->edges().end(); ++it){
      if ( typeName.empty() || typeid(**it).name()==typeName)
        (*action)(*it, params);
    }
  }

} // end namespace