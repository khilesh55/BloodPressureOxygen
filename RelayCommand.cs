using Prism.Commands;
using System;
using System.Windows.Input;

namespace SpasticityClient
{
    public static class ApplicationCommands
    {
        public static CompositeCommand ReadCommand = new CompositeCommand();
    }

    public class RelayCommand : ICommand
    {
        private Action _action;

        public RelayCommand(Action action)
        {
            _action = action;
        }

        public bool CanExecute(object parameter)
        {
            return true;
        }

        public void Execute(object parameter)
        {
            _action();
        }

        public event EventHandler CanExecuteChanged { add { } remove { } }
    }
}
